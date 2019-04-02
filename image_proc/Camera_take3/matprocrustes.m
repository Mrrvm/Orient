function [R, T] = matprocrustes(m1, m2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2   2D points before and after
%   m1,m2    2D points before and after
%            transformation
% Output
%   R       Rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

radius = 1;
M1 = project_to_sphere(m1, radius);
M2 = project_to_sphere(m2, radius);

[d,Z,tr] = procrustes(M1', M2', 'scaling', false, 'reflection', false); 

T = tr.c(1,:)';
R = tr.T;

end