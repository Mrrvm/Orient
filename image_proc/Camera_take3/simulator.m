function [M1, M2, m1, m2] = simulator(N, d, R, T)
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

% Generate random 3D points
M1 = (d-0.05)*rand([3,N*5])+0.05;

% Rotate points
M2 = R*M1;
% Translate points
M2 = M2+T;

keep = M2(3, :) > 0;

M1 = M1(:, keep);
M2 = M2(:, keep);

M1 = M1(1:3, 1:N); 
M2 = M2(1:3, 1:N); 

m1 = project_to_plane(M1);
m2 = project_to_plane(M2);

end

