function [eRoppr, eRfpro, eRmbpe, eRepog] = estimator(m1, m2, radius, K, B, eul)
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

%% Run minimization of back projection error
[Rmbpe, Tmbpe] = minBackProject(m1, m2, B, rotm2eul(Roppr), radius, K);

%% Run full procrustes
[Rfpro, Tfpro] = fullProcrustes(m1, m2, radius, K);

REPS = 100;
for i=1:REPS
tic;
%% Run epipolar geometry approach
[Repog, Tepog] = epipolarGeo(m1, m2, radius, K);
end
averageTime = toc/REPS

%% Compute error between each method results and truth
eRoppr = norm(rotm2eul(Roppr)-eul);
eRfpro = norm(rotm2eul(Rfpro)-eul);
eRmbpe = norm(rotm2eul(Rmbpe)-eul);
eRepog = norm(rotm2eul(Repog)-eul);
 
end
