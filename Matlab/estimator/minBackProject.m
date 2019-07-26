function [R, T] = minBackProject(m1, m2, B, eulinit, radius, K)
%minBackProject Minimization of Back Projection Error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2    2D points before and after
%                  transformation
%   B             Baseline
%   eulinit     Initialization for euler angles
%   radius     Sphere radius
%   K             Intrinsics matrix
% Output
%   R,T          Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = [1 0 0; 0 1 0; 0 0 1];
M1 = projectToSphere(K, m1, radius);
Zinit = M1(3,:);

options = optimset('MaxFunEvals', 100, 'MaxIter', 100);%, 'PlotFcns',@optimplotfval);
[x,fval,exitflag,output] = fminsearch(@(x)objectiveFunMBPE(x, m1, m2, B, K), [eulinit Zinit], options);

R = eul2rotm(x(1:3));
Z1 = x(4:end);
T = (R-I)*B;

end
