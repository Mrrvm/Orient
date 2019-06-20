function [eRoppr, eRfpro, eRmbpe, eRepog] = runSimulation(angles, radius, K, nMatches, maxD, minD, B, nAngles, nPixels, imgDim)
%runSimulation Simulate points on space and estimate transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   angles        Set of angles to test
%   radius        Sphere radius
%   K                Intrinsics matrix
%   nMatches   Number of matches per sample
%   maxD        Max distance to camera
%   minD         Min distance to camera
%   B                Baseline
%   nAngles     Number of different angles to try
%   nPixels      Number of pixels to deviate in noise
% Output
%   eR...          Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I = [1 0 0; 0 1 0; 0 0 1];

% x-1, y-2, z-3 
for i=1:3
    for j=1:nAngles
        %% Simulate points
        R = eul2rotm(-angles(j)*[(3-i)==0 (2-i)==0 (1-i)==0]);
        T = (R-I)*B;
        [Mw, M1, M2, m1, m2, err] = simulator(nMatches, R, T, maxD, minD, B, K, imgDim);
        if err == 1
            continue;
        end
        [m1, m2] = noiseGen(m1, m2, nMatches, nPixels);
        %% Estimate transformation error
        [eRoppr(i,j), eRfpro(i,j), eRmbpe(i,j), eRepog(i,j)]= estimator(m1, m2, radius, K, B, rotm2eul(R));
    end  
end
 
end