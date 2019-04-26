function [R, T] = fullProcrustes(m1, m2, radius, K)
%fullProcrustes Procrustes analysis with SVD and 
% point standardization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2    2D points before and after
%            transformation
%   radius   Sphere radius
%   K        Intrinsics matrix
% Output
%   R,T      Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ki = inv(K);
M1 = projectToSphere(m1, radius);
M1 = (Ki*M1)';
M2 = projectToSphere(m2, radius);
M2 = (Ki*M2)';

mu1 = mean(M1, 1);
mu2 = mean(M2, 1);

N1 = M1 - repmat(mu1, size(M1, 1), 1);
N2 = M2 - repmat(mu2, size(M1, 1), 1);

ssq1 = sum(N1.^2,1);
ssq2 = sum(N2.^2,1);
ssq1 = sum(ssq1);
ssq2 = sum(ssq2);
norm1 = sqrt(ssq1);
norm2 = sqrt(ssq2);
    
N1 = N1 / norm1;
N2 = N2 / norm2;

A = N1'*N2;
[U,S,V] = svd(A);

R = V*U';
T = mu1 - mu2*R;
end

