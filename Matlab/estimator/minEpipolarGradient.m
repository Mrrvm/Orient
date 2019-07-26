function [Rgrat, Tgrat] = minEpipolarGradient(m1, m2, B, K, eulinit)
%minEpipolarGradient Minimization of Epipolar Gradient
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2    2D points before and after
%                  transformation
%   B             Baseline
%   Rinit        Initialization for the rotation matrix
%   radius     Sphere radius
%   K             Intrinsics matrix
%   Roppr     Procrustes result
% Output
%   R,T          Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = [1 0 0; 0 1 0; 0 0 1];

options = optimset('MaxFunEvals', 100, 'MaxIter', 100);
[x,fval,exitflag,output] = fminsearch(@(x)objectiveFunGRAT(x, m1, m2, B, K), eulinit, options);% Finit(:), options);

Rgrat = eul2rotm(x(1:3));
Tgrat = (Rgrat-I)*B;

end

