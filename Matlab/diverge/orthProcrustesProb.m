function [R, T] = orthProcrustesProb(m1, m2, radius, K)
%orthProcrustesProb Estimate using only orthogonal 
% procrustes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2  2D points before and after
%                transformation
%  radius    Sphere radius
%  K            Intrinsics matrix
% Output
%  R            Rotation matrix
%  T            Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


M1 = projectToSphere(K, m1, radius);
M2 = projectToSphere(K, m2, radius);

A = M1*M2';
[U,S,V] = svd(A);

R = V*U';

T = [0 0 0]';

end
