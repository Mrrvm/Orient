function [eR, eT] = runSimulation(angles, radius, K, nMatches, maxD, minD, B, nAngles, nPixels, imgDim, methods)
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
        [m1, m2] = noiseGen(m1, m2, nPixels);
        %% Estimate transformation error
        [eRi, eTi]= estimator(m1, m2, radius, K, B, rotm2eul(R), T, methods);
        sizeeRi = size(eRi, 2);
        for n=1:sizeeRi
            eR(3*(n-1)+i, j) = eRi(n); 
            eT(3*(n-1)+i, j) = eTi(n); 
        end
    end  
end
 
end