function [Rmee Tmee] = minEpipolarError(m1, m2, B, K, eulinit, N1, N2)
%minEpipolarError Minimization of Epipolar Error
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

options = optimset('MaxFunEvals', 100, 'MaxIter', 100);%, 'PlotFcns',@optimplotfval);
[x,fval,exitflag,output] = fminsearch(@(x)objectiveFunMEE(x, m1, m2, B, K, N1, N2), eulinit);%, options);% Finit(:), options);

Rmee = eul2rotm(x(1:3));
Tmee = (Rmee-I)*B;

end

