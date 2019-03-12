function f = objectivefun(x, m1, m2, B, N)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   x(1:9) = R       Rotation matrix
%   x(10:N+10) = Z1  Depth
%   m1,m2            2D points
%   B                Baseline 
% Notes
%   intrinsics will be considered later 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = [1 0 0; 0 1 0; 0 0 1];

R(1, 1:3) = x(1:3);
R(2, 1:3) = x(4:6);
R(3, 1:3) = x(7:9);
Z1 = x(10:end);

T = (R-I)*B;
Rt = R';

m1h = [m1; ones(1,N)];
m2h = [m2; ones(1,N)];

aux = R*(Z1.*m1h)+T;
Z2 = aux(3,:);

m2e = (aux)./Z2;
m1e = (Rt*(Z2.*m2h)-Rt*T)./Z1;

u1d = m1e(1,:) - m1(1,:);
v1d = m1e(2,:) - m1(2,:);
u2d = m2e(1,:) - m2(1,:);
v2d = m2e(2,:) - m2(2,:);

f = sum(u1d.*u1d + u2d.*u2d + v1d.*v1d + v2d.*v2d);

end