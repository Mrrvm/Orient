function [R, T] = orthProcrustesProb(m1, m2, radius, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimate using only orthogonal procrustes
% Input
%   m1,m2    2D points before and after
%            transformation
%  radius    Sphere radius
%  K         Intrinsics matrix
% Output
%   R       Rotation matrix
%   T       Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kt = K';
M1 = projectToSphere(m1, radius);
M1 = Kt*M1;
M2 = projectToSphere(m2, radius);
M2 = Kt*M2;

A = M1*M2';
[U,S,V] = svd(A);

R = V*U';

T = [0 0 0]';

end
