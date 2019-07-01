function f = objectiveFunMEE(x, m1, m2, B, K)
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
F = inv(K)'*Tx*R*inv(K);

m1h = [m1; ones(1, nMatches)];
m2h = [m2; ones(1, nMatches)];

l2 = F*m1h;
l1 = F'*m2h;

aux2 = [1 0 0; 0 1 0]*l2;
aux1 = [1 0 0; 0 1 0]*l1;

norm2 = sqrt(aux2(1, :).*aux2(1, :) + aux2(2, :).*aux2(2, :));
norm1 = sqrt(aux1(1, :).*aux1(1, :) + aux1(2, :).*aux1(2, :));

l2 = l2./norm2;
l1 = l1./norm1;

err2 = m2h(:)'*l2(:);
err1 = m1h(:)'*l1(:);

f = err1 + err2;

end