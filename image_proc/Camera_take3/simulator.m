function [m1, m2] = simulator(N, d, R, T)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   angles  Rotation angles per axis
%   T       Translation
%   N       Number of points
%   d       distance from the camera
% Output
%   m1,m2   2D points before and after 
%            transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Determine max deviation
mdev = 2*d;

% Generate random 3D points
M1(1, :) = mdev*rand(N,1);
M1(2, :) = mdev*rand(N,1);
M1(3, :) = d*rand(N,1);

% Rotate points
M2 = R*M1;
% Translate points
M2 = M2+T;

m1 = project_to_plane(M1);
m2 = project_to_plane(M2);

end

