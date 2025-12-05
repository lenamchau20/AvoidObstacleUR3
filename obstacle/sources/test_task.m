% test_task1.m

%% Section 1: Create world and define obstacles
clc; clear; close all;
folder = fileparts(which('test_task.m')); 
addpath(genpath(fullfile(folder, 'library')));

initialPosition = [0.25; 0.26; 0.29];
myWorld = EnvironmentUR3(initialPosition);

% add cylinder
myWorld.addCylinder(0.07, [0.3, 0.1, 0.4],5);   %ok %radius, pos,rho, vel

% add Ellipsoid
myWorld.addEllipsoid([0.07, 0.07, 0.07], [0.3, -0.1, 0.45],0.01); %ok

%  Draw Isosurfaces
%myWorld.showGammaIsosurfaces([1.0, 1.2, 1.5], 0)  % 0: dung hàm gamma
                                                   % 1 = dung gammaDistance

%  vận tốc hằng theo z
% vel_func1 = @(x,t) [0; 0; 0.1];
% myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.3, -0.1, 0.35], 0.5, vel_func1);

%% Section 2: Simulate initial conditions

% Initial state: joint velocity and position
q = myWorld.robot.computeInverseKinematics(initialPosition, myWorld.robot.robot.homeConfiguration, ...
                false, diag([1, -1, -1]));
q_dot = zeros(length(q), 1);

% Create robot and controller structure
attractor = [0.3; -0.3; 0.4];
dsModulated = @(x) modulatedDS(x, attractor, myWorld);

% Display initial robot pose
ax = show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');

% Plot target point and DS open-loop path
hold on;
plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 20);

opt_sim = []; 
opt_sim.plot  = 0;
opt_sim.dt = 3e-3;
opt_sim.i_max = 1500;
echo off;
[x_sim, ~] = Simulation(initialPosition , [], dsModulated, opt_sim);
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'r', 'LineWidth', 2);

% Plot initial velocity and position
desired_vel = dsModulated(initialPosition);
plot3(initialPosition(1), initialPosition(2), initialPosition(3), '.k', 'MarkerSize', 10)
quiver3(initialPosition(1), initialPosition(2), initialPosition(3), ...
        desired_vel(1), desired_vel(2), desired_vel(3), 0.1, 'b'); 


% Plot DS open-loop path starting at random points
axisLimit = 0.9*myWorld.axisLimit;
for iTraj = 1:30
    randomPosition = rand(3, 1);
    randomPosition(1) = randomPosition(1)*(axisLimit(2)-axisLimit(1)) + axisLimit(1);
    randomPosition(2) = randomPosition(2)*(axisLimit(4)-axisLimit(3)) + axisLimit(3);
    randomPosition(3) = randomPosition(3)*(axisLimit(6)-axisLimit(5)) + axisLimit(5);
    scatter3(randomPosition(1), randomPosition(2), randomPosition(3))
    [~, x_sim, ~] = evalc('Simulation(randomPosition , [], dsModulated, opt_sim)');

    plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'Color', [1,0.6,0.2,0.6]);
end

drawnow;


%% Section 3: Simulate closed-loop trajectory

% Setup controller and disturbance
ctrl = LimitedJointVelocityController(500, myWorld.robot.robot); 
disturbance = CartesianForceDisturbance(5e3, 2);

% Simulation setup and parameters
% text(axisLimit(2), axisLimit(4), 0.5 * axisLimit(6),{'\bf Perturbations: ', '\rm A/D keys : X axis ', 'Q/E keys : Y axis ', 'W/S keys : Z axis ', ...
%                                                      '', '\bfStop simulation : \rm Space bar'}, 'FontSize', 12);


% Tắt các interaction mode để được phép gán KeyPressFcn
%zoom(myWorld.figure,'off'); pan(myWorld.figure,'off'); rotate3d(myWorld.figure,'off'); datacursormode(myWorld.figure,'off');
% Gán callback phím -> cập nhật lực vào 'disturbance.values'
set(myWorld.figure,'WindowKeyPressFcn', {@KeyboardCb, disturbance});

delta_t = 25e-3; %bước time mô phỏng
start = tic;
i = 0;
position = myWorld.robot.computeForwardKinematics(q);
global stopFlag; %global variable
stopFlag = false; %chưa dừng mô phỏng
while vecnorm(position - attractor) > 0.01 && ~stopFlag
    % Display disturbance
    if norm(disturbance.values) > 0 && disturbance.iteration < disturbance.nb_active_iterations
        if disturbance.iteration == 0
            if exist('hline', 'var') && exist('hhead', 'var')
                delete(hline);
                delete(hhead);
            end
            [hline, hhead] = arrow3d(position', position' + 0.5 * disturbance.values'/norm(disturbance.values), ...
                15, 'cylinder', [0.2, 0.2], [20, 10], [1, 0 ,0; 1, 0, 0]);
            drawnow;
        end
        disturbance.iteration = disturbance.iteration + 1;
    else
        disturbance.values = zeros(3,1);
        disturbance.iteration = inf;
    end

    % Integrate forward given current state and control
    [t, y] = ode15s(@(t,y) forwardStateDynamic(t, y, myWorld.robot.robot, dsModulated, disturbance.values, ctrl), ...
        [i * delta_t, (i+1) * delta_t], [q; q_dot]);
    
    q = y(end,1:6)';
    q_dot = y(end,7:12)';
    
    %show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);
    show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
    plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10)
    drawnow;
    
    position = myWorld.robot.computeForwardKinematics(q);
    myWorld.endEffectorPosition = position;
    
    % Plot desired velocity from DS
    if mod(i, 4)==0
        desired_vel = dsModulated(position);
        quiver3(position(1), position(2), position(3), desired_vel(1), desired_vel(2), desired_vel(3), 0.2, 'Color', [0.2,0,1,0.9]);   
    end

    % Move obstacles if desired
    for k =1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(i*delta_t)
    end

    i = i + 1;
end
disp("Finished Simulation On the Target")

T0_ee = myWorld.robot.robot.getTransform(q, 'ee_link')
time_elapsed = toc(start);
disp("Average computation time: " + time_elapsed / i + " [s] | Total time: " + time_elapsed + " [s]")


% Simulation function
function dydt = forwardStateDynamic(t, y, robot, ds_control, force, ctrl)
    
    % Current state
    q = y(1:6);
    q_dot = y(7:12);

    % Desired state Từ q -> vị trí end-effector
    current_position = tform2trvec(robot.getTransform(q, 'ee_link'))';
    desired_velocity = ds_control(current_position);   %DS đã modulation cho vị trí hiện tại

    % Compute corresponding control torque (PD controller) 
    full_jac = robot.geometricJacobian(q, 'ee_link');
    controlAcceleration = ctrl.computeCommand(desired_velocity, q_dot, full_jac);

    % Compute disturbance
    torque_disturbance = full_jac(4:end,:)' * force;
    
    % Compute state derivative from current state and control
    dydt = [q_dot; ...
            controlAcceleration + torque_disturbance];
end


function KeyboardCb(~, event, force_disturbance)
    global stopFlag;
    if strcmp(event.Key, 'space') % space bar
        stopFlag = true;
        disp("Stopped Simulation")
    end
    force_disturbance.handleKeyEvent(event.Key);
    disp(['Pressed key: ', event.Key]);
    
end

function xdot = modulatedDS(x, attractor, world)

    
    xdot_nominal = -3*eye(3)*(x-attractor);
   
    % Initialize modulation matrix and average obstacle velocity
    M_tot = eye(3);
    %meanObstacleVelocity = zeros(3, 1);

    %% ------ Write your code below ------ %%
    %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %
    
    % Create the array of gamma values from the list world.listOfObstacles
    % for convenience
    kObst = length(world.listOfObstacles);
    %gamma = zeros(1, kObst);
    
    gradient_gamma = zeros(3, kObst);
    
    %rho=0.3;
    rho = zeros(1, kObst);
    
    ree = world.endEffectorRadius; %part2 task 2.3
    g = zeros(1, kObst);
    gamma_eff = zeros(1, kObst);

    for k = 1:kObst
        % Get the gamma value of obstacle k at point x
        %gamma(k) =  world.listOfObstacles(k).gamma(x(1), x(2), x(3));
        %gamma(k) =  world.listOfObstacles(k).gammaDistance(x(1), x(2), x(3));
        g(k) = world.listOfObstacles(k).gammaDistance(x(1), x(2), x(3)); %part2 task 2.3

        % Get the gradient of obstacle k at point x
        gradient_gamma(:,k)=world.listOfObstacles(k).gradientGamma(x(1), x(2), x(3));
        
        % lấy rho riêng của obstacle
        rho(k) = world.listOfObstacles(k).rho; 

        % Áp dụng margin r_ee theo part2 task 2.3
        if g(k) > 1 + ree
            gamma_eff(k) = g(k) - ree;
        else
            gamma_eff(k) = 1;
        end
    end

    %tính gminus1 sau khi lấy gamma
    eps_d = 1e-9;                   % tranh chia cho 0
    %gminus1 = max(gamma - 1, eps_d);% vector: gamma^i(x) - 1 cho moi i
    gminus1 = max(gamma_eff - 1, eps_d); %part2 task 2.3

    % Go over each obstacle, compute the modulation matrix and apply it to
    % the initial ds velocity xdot_modulated
    for k = 1:kObst   
        % Task 2: Compute weight function 
        if kObst > 1
            gk   = gminus1(k);  % vector tất cả gi - 1
            gi   = gminus1;
            frac = gi ./ (gk + gi);
            frac(k) = 1; 
            weight = prod(frac);
        else
            weight = 1;
        end
        
        % Compute modulation matrix 
         %D = diag([ 1 - 1/(gamma(k)), 1 + 1/(gamma(k)), 1 + 1/(gamma(k))]); %task1
         %D = diag([ 1 - 1*weight/(gamma(k))^(1/rho(k)), 1 + 1*weight/(gamma(k))^(1/rho(k)), 1 + 1*weight/(gamma(k))^(1/rho(k))]); 
        D = diag([ 1 - 1*weight/(gamma_eff(k))^(1/rho(k)), 1 + 1*weight/(gamma_eff(k))^(1/rho(k)), 1 + 1*weight/(gamma_eff(k))^(1/rho(k))]);  %part2 task 2.3
        
        % Compute normal and tangent on ellipse
        
        %normal = gradient_gamma(:,k) / norm(gradient_gamma(:,k));
        normal = gradient_gamma(:,k);                          
        normal = normal / (norm(normal));              
        normal = normal(:);
        P = eye(3) - normal*normal.';
        
        tangent1 = P * [1;0;0];
        tangent1 = tangent1 / norm(tangent1);
        tangent2 = cross(normal, tangent1);
        tangent2 = tangent2 / norm(tangent2);
    
        E = [normal, tangent1, tangent2];

        Mk = E * D * E';
        M_tot = Mk * M_tot;
    end    

    xdot = M_tot * xdot_nominal;
    
end
