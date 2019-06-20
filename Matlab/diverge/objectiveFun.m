function f = objectiveFun(x, m1, m2, B, K)
%objectiveFun Objective function for MBPE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   x(1:9) = R             Rotation matrix
%   x(10:N+10) = Z1  Depth
%   m1,m2                 2D points
%   B                          Baseline 
%   K                          Intrinsics matrix
% Output
%   f                           Min value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = [1 0 0; 0 1 0; 0 0 1];

nMatches = size(m1, 2);

R = eul2rotm(x(1:3));
Z1 = x(4:end);
T = (R-I)*B;
Rt = R';

m1h = [m1; ones(1,nMatches)];
m2h = [m2; ones(1,nMatches)];

M2 = R*(Z1.*(K\m1h))+T;
Z2 = M2(3,:);
m2e = projectToPlane(K, M2); 

M1 = Rt*(Z2.*(K\m2h))-Rt*T;
m1e = projectToPlane(K, M1);

u1d = m1e(1,:) - m1(1,:);
v1d = m1e(2,:) - m1(2,:);
u2d = m2e(1,:) - m2(1,:);
v2d = m2e(2,:) - m2(2,:);

f = sum(u1d.*u1d + u2d.*u2d + v1d.*v1d + v2d.*v2d + 10000000*heaviside(-Z2) + 1000000000*heaviside(-Z1));

%fprintf(fid,'%f %f %f %f;\n', u1d, v1d, u2d, v2d);

%f = sum(sqrt(u1d.*u1d + v1d.*v1d)+ sqrt(u2d.*u2d + v2d.*v2d));

% M2e = R*M1+T;
% M1e = Rt*M2+Rt*T;
% f = sum(sqrt(sum((M2-M2e).^2,1))) + sum(sqrt(sum((M1-M1e).^2,1)));

end