function [R, T] = orthProcrustesProb(m1, m2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimate using only orthogonal procrustes
% Input
%   m1,m2    2D points before and after
%            transformation
% Output
%   R       Rotation matrix
%   T       Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

radius = 1;
M1 = projectToSphere(m1, radius);
M2 = projectToSphere(m2, radius);

A = M1*M2';
[U,S,V] = svd(A);

R = V*U';

T = [0 0 0]';

end
