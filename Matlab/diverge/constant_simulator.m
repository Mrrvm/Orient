function [M1, M2, m1, m2] = constant_simulator(Mw, nMatches, R, T, maxD, minD, B, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%simulator Simulate two views of a 3D scene
% Input
%   nMatches Number of points
%   R              3D points rotation to make
%   T              Translation associated to the baseline
%   maxD       Max distance to the camera
%   minD        Min distance to the camera
%   B              Baseline
%   K              Intrinsics matrix
%   Mw           3D world points pre-transformation
% Output
%   M2           3D points pos transformation
%   m1,m2     2D points pre and pos-transformation
%   err           Error of generation success
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Transform from world to camera 1
M1 = Mw - B;

% Rotate points
M2 = R*M1+T;

m1 = projectToPlane(K, M1);
m2 = projectToPlane(K, M2);

end

