function [eRoppr, eRmee, eRmbpe, eRn8p] = estimator(m1, m2, radius, K, B, eul)
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

%% Run minimization of epipolar error
[Rmee, Tmee] = minEpipolarError(m1, m2, B, radius, K, rotm2eul(Roppr));

%% Run epipolar geometry norm8point approach
[Rn8p, Tn8p] = recoverPoseFromF(norm8point(m1, m2) , K, m1(:,1), m2(:,1), radius);

%% Run epipolar geometry linear approach


%% Compute error between each method results and truth
eRoppr = norm(rotm2eul(Roppr)-eul);
eRmee = norm(rotm2eul(Rmee)-eul);
eRmbpe = norm(rotm2eul(Rmbpe)-eul);
eRn8p = norm(rotm2eul(Rn8p)-eul);
 
end
