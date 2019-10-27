function [eR, eT, rot] = estimator(m1, m2, radius, K, B, rreal, treal, methods)
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
I = [1 0 0; 0 0 1; 0 0 1];

for j=1:nMethods
    
    if strcmp(methods(j), 'OPPR')          
        [R, T] = orthProcrustesProb(m1, m2, radius, K);
    end
    
    if strcmp(methods(j), 'MEE')
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        [R, T] = minEpipolarError(m1, m2, B, K, rotm2eul(Roppr), I, I);
    end
    
    if strcmp(methods(j), 'MEEN')  
        N1 = getNormalizationMatrix(m1);
        N2 = getNormalizationMatrix(m2);
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        [R, T] = minEpipolarError(m1, m2, B, K, rotm2eul(Roppr), N1, N2);
    end
    
    if strcmp(methods(j), 'MBPE')  
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        [R, T] = minBackProject(m1, m2, B, rotm2eul(Roppr), radius, K);     
    end
    
    if strcmp(methods(j), 'N8P')
        F = norm8point(m1, m2, N1, N2);
        F = adjustFFrobeniusNorm(F);
        F = N2'*(F*N1); % Denormalize
        [R, T] = recoverPoseFromF(F , K, m1(:,1), m2(:,1), radius);
    end
    
    if strcmp(methods(j), 'GRAT') 
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        [R, T] = minEpipolarGradient(m1, m2, B, K, rotm2eul(Roppr));
    end
    
    % Compute error between each method results and truth
    rot(j, :) = -rotm2eul(R);
    eR(j) = norm((-rotm2eul(R))-rreal);
    eT(j) = norm(T-treal);
    clear R T;
   
end
 
end
