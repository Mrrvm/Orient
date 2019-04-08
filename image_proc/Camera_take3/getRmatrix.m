function R = getRmatrix(angles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtain rotation matrix from angles XYZ
% Input
%   angles     Angles of rotation per axis
% Output
%   R          Rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax = angles(1);
ay = angles(2);
az = angles(3);

Rx = [1 0       0; 
      0 cos(ax) -sin(ax); 
      0 sin(ax) cos(ax)];

Ry = [cos(ay)  0 sin(ay); 
      0        1 0; 
      -sin(ay) 0 cos(ay)];
  
Rz = [cos(az) -sin(az) 0; 
      sin(az) cos(az)  0; 
      0       0        1];
  
R = Rx*Ry*Rz;
 
end

