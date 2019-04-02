function [R, T] = opprocrustes(m1, m2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2    2D points before and after
%            transformation
% Output
%   R       Rotation matrix
%   T       Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

radius = 1;
M1 = project_to_sphere(m1, radius);
M2 = project_to_sphere(m2, radius);

A = M1*M2';
[U,S,V] = svd(A);

R = V*U';

T = [0 0 0]';

end
