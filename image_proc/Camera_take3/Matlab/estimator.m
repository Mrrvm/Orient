function [eRoppr, eRfpro, eRmbpe, eRepog] = estimator(m1, m2, radius, K, B)
%estimator Estimate transformation based on 4 different methods: 
% orthogonal procrustes problem, full procrustes, minimization of back
% projection error and epipolar geometry 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate points on space and estimate transformation
% Input
%   angles      Set of angles to test
%   radius      Sphere radius
%   K               Intrinsics matrix
%   nMatches  Number of matches per sample
%   maxD       Max distance to camera
%   minD        Min distance to camera
%   B               Baseline
%   nAngles    Number of different angles to try
%   nPixels     Number of pixels to deviate in noise
% Output
%   eR...         Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run orthogonal procrustes problem
[Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);

%% Run full procrustes
[Rfpro, Tfpro] = fullProcrustes(m1, m2, radius, K);

%% Run minimization of back projection error
[Rmbpe, Tmbpe] = minBackProject(m1, m2, B, Roppr, radius, K);

%% Run epipolar geometry approach
[Repog, Tepog] = epipolarGeo(m1, m2, radius, K);

%% Compute error between each method results and truth
r      = matrixToAxisAngle(R);
roppr  = matrixToAxisAngle(Roppr);
rfpro  = matrixToAxisAngle(Rfpro);
rmbpe  = matrixToAxisAngle(Rmbpe);
repog  = matrixToAxisAngle(Repog);
eRoppr(i,j)  = norm(r-roppr);
eRfpro(i,j)  = norm(r-rfpro);
eRmbpe(i,j)  = norm(r-rmbpe);
eRepog(i,j)  = norm(r-repog);
eToppr(i,j)  = norm(T-Toppr);
eTfpro(i,j)  = norm(T-Tfpro);
eTmbpe(i,j)  = norm(T-Tmbpe);
eTepog(i,j)  = norm(T-Tepog);
 
end