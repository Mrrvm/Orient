function f = objectiveFunGRAT(x, m1, m2, B, K)
%objectiveFun Objective function for GRAT
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

epiEq = sum(m2h.*(F*m1h));

l2 = F*m1h;
l1 = F'*m2h;

aux1 = [1 0 0; 0 1 0]*l1;
aux2 = [1 0 0; 0 1 0]*l2;

var = aux2(1, :).*aux2(1, :) + aux2(2, :).*aux2(2, :) + ...
    aux1(1, :).*aux1(1, :) + aux1(2, :).*aux1(2, :);


f = sum((epiEq.^2)/var);


end