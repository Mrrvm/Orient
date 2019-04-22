function [R, T] = minBackProject(m1, m2, B, N, R_init, radius, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimization of Back Projection Error
% Input
%   m1,m2    2D points before and after
%            transformation
%   B        Baseline
%   N        Number of point matches
%   R_init   Initialization for the 
%            rotation matrix
%   radius   Sphere radius
%   K        Intrinsics matrix
% Output
%   R,T      Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = [1 0 0; 0 1 0; 0 0 1];
Rinit = [R_init(1,1:3) R_init(2,1:3) R_init(3,1:3)];
M1 = projectToSphere(m1, radius);
Zinit = M1(3,:);

options = optimset('MaxFunEvals',10000000000);
[x,fval,exitflag,output] = fminsearch(@(x)objectiveFun(x, m1, m2, B, N, K), [Rinit, Zinit], options);

R(1, 1:3) = x(1:3);
R(2, 1:3) = x(4:6);
R(3, 1:3) = x(7:9);
Z1 = x(10:end);
T = (R-I)*B;

end
