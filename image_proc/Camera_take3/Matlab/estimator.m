function [eRoppr, eRfpro, eRmbpe, eRepog] = estimator(m1, m2, radius, K, B, r)
%estimator Estimate transformation error based on 4 different methods: 
% orthogonal procrustes problem, full procrustes, minimization of back
% projection error and epipolar geometry 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   radius       Sphere radius
%   K               Intrinsics matrix
%   B               Baseline
%   r               Ground truth
% Output
%   eR...          Error from each method
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
roppr  = matrixToAxisAngle(Roppr);
rfpro   = matrixToAxisAngle(Rfpro);
rmbpe = matrixToAxisAngle(Rmbpe);
repog  = matrixToAxisAngle(Repog);
eRoppr  = norm(r-roppr);
eRfpro   = norm(r-rfpro);
eRmbpe = norm(r-rmbpe);
eRepog  = norm(r-repog);
 
end