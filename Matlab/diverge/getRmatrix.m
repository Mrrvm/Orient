function R = getRmatrix(angles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtain rotation matrix from angles XYZ
% Input
%   angles   Angles of rotation
% Output
%   R          Rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Rx = [1 0       0; 
         0 cos(angles(1)) -sin(angles(1)); 
         0 sin(angles(1)) cos(angles(1))];

Ry = [cos(angles(2))  0 sin(angles(2)); 
         0        1 0; 
        -sin(angles(2)) 0 cos(angles(2))];
  
Rz = [cos(angles(3)) -sin(angles(3)) 0; 
         sin(angles(3)) cos(angles(3))  0; 
         0       0        1];
  
R = Rz*Ry*Rx;
 
end

