function [eRoppr, eRfpro, eRmbpe, eRepog] = runSimulation(angles, radius, K, nMatches, maxD, minD, B, nAngles, nPixels)
%runSimulation Simulate points on space and estimate transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   angles     Set of angles to test
%   radius     Sphere radius
%   K          Intrinsics matrix
%   nMatches   Number of matches per sample
%   maxD       Max distance to camera
%   minD       Min distance to camera
%   B          Baseline
%   nAngles    Number of different angles to try
%   nPixels    Number of pixels to deviate in noise
% Output
%   eR...      Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I = [1 0 0; 0 1 0; 0 0 1];

% x-1, y-2, z-3 
for i=1:3
    for j=1:nAngles
        %% Simulate points
        R = getRmatrix(angles((i-1)*j+j,:));
        T = (R-I)*B;
        [M1, M2, m1, m2, err] = simulator(nMatches, R, maxD, minD, B, K);
        if err == 1
            continue;
        end
        %showScenario(M1, M2, B, R, maxD);
        [m1, m2] = noiseGen(m1, m2, nMatches, nPixels);
        %% Estimate transformation error
        [eRoppr(i,j), eRfpro(i,j), eRmbpe(i,j), eRepog(i,j) ]= estimator(m1, m2, radius, K, B, R);
    end  
end
 
end