%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2024 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email: aude.billard@epfl.ch
%   website: lasa.epfl.ch
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 3 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef VerticalCylinder < Shapes %khai báo class nó kế thức (<) class Shapes
   properties
      radius

   end

   methods
    
       function self = VerticalCylinder(radius, position, velocity_function, rho)
                                          %r: bk trụ, position: tâm trụ
                                          %velo_func: trụ di chuyển 
                                          %rho: tham số nhạy DS modulation
           param = struct();
           param.radius = radius;

           %% ------ Write your code below ------
           %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

           % Fill the gamma functions and the gradient
           % The parameter param.radius is the radius of the cylinder
           gamma = @(x, y, z, param) sqrt(x.^2 + y.^2) ./ param.radius;
           gradientGamma = @(x, y, z, param) [x ./ (param.radius .* sqrt(x.^2 + y.^2)); y ./ (param.radius .* sqrt(x.^2 + y.^2));0.*z];

           % To complete in TASK 4
           %gammaDistance = @(x, y, z, param) (x.^2 + y.^2) / param.radius.^2;
           % ===== Task 4: Gamma theo khoảng cách Euclid: gamma_dist = 1 + d ===== %nếu muốn gọi isoValue là 1.0 1.1 1.3 thì bỏ 1 + đi
           % d = max( sqrt((x-xc)^2+(y-yc)^2) - r, 0 )
           gammaDistance = @(x, y, z, param) 1 + max( sqrt( (x.^2 + y.^2 )) - param.radius, 0 ); 
           gradientgammaDistance = @(x,y,z,param) [2.*x./(param.radius.^2); 2.*y./(param.radius.^2); 0.*z];
           %  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
           %% ------ Write your code above ------
            %Tạo đối tượng VerticalCylinder bằng cách gọi constructor Shapes(...)
           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho)

           self.radius = radius; %lưu bán kinh

       end

   end

end
