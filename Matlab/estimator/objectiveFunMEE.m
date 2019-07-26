function f = objectiveFunMEE(x, m1, m2, B, K, N1, N2)
%objectiveFun Objective function for MEE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   x(1:3) = R             Euler Angles
%   m1,m2                 2D points
%   B                          Baseline 
%   K                          Intrinsics matrix
% Output
%   f                           Min value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = [1 0 0; 0 1 0; 0 0 1];
nMatches = size(m1, 2);

R =  eul2rotm(x(1:3));
T = (R-I)*B;
Tx = [0 -T(3) T(2); T(3) 0 -T(1); -T(2) T(1) 0];
F = N2'*((inv(K)'*Tx*R*inv(K))*N1);

m1h = [m1; ones(1, nMatches)];
m2h = [m2; ones(1, nMatches)];

m1n = N1*m1h;
m2n = N2*m2h;

l2 = F*m1n;
l1 = F'*m2n;

aux1 = [1 0 0; 0 1 0]*l1;
aux2 = [1 0 0; 0 1 0]*l2;

norm1 = sqrt(aux1(1, :).*aux1(1, :) + aux1(2, :).*aux1(2, :));
norm2 = sqrt(aux2(1, :).*aux2(1, :) + aux2(2, :).*aux2(2, :));

l1 = l1./norm1;
l2 = l2./norm2;

err1 = m1h(:)'*l1(:);
err2 = m2h(:)'*l2(:);

f = err1.^2 + err2.^2;

end