function [eR, eT] = estimator(m1, m2, radius, K, B, rreal, treal, methods)
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

nMethods = size(methods, 2);
[Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);

for j=1:nMethods
    
    if strcmp(methods(j), 'OPPR')
        %% Run orthogonal procrustes problem
        R = Roppr;
        T = Toppr;
    end
    if strcmp(methods(j), 'MBPE') 
        %% Run minimization of back projection error
        [R, T] = minBackProject(m1, m2, B, rotm2eul(Roppr), radius, K);
    end
    if strcmp(methods(j), 'MEE')
        %% Run minimization of epipolar error
        [R, T] = minEpipolarError(m1, m2, B, radius, K, rotm2eul(Roppr));
    end
    if strcmp(methods(j), 'N8P')
        %% Run epipolar geometry norm8point approach
        [R, T] = recoverPoseFromF(norm8point(m1, m2) , K, m1(:,1), m2(:,1), radius);
    end
    if strcmp(methods(j), 'GRAT') 
        %% Run epipolar geometry gradient technique
        [R, T] = minEpipolarGradient(m1, m2, B, K, rotm2eul(Roppr));
    end
    
    %% Compute error between each method results and truth
    eR(j) = norm(rotm2eul(R)-rreal);
    eT(j) = norm(T-treal);
    clear R T;
   
end
 
end
