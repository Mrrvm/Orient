function [plotAng, axisCount, eR, eT, ransacRes] = runSimulation(angles, vars)
%runSimulation Simulate points on space and estimate transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   angles               Set of angles to test
%   radius               Sphere radius
%   K                       Intrinsics matrix
%   nMatches           Number of matches per sample
%   nFalseMatches   Number of false matches per sample
%   maxD                Max distance to camera
%   minD                Min distance to camera
%   B                       Baseline
%   nAngles             Number of different angles to try
%   nPixels             Number of pixels to deviate in noise
% Output
%   eR...                 Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I = [1 0 0; 0 1 0; 0 0 1];
axisCount = zeros(1,3);
ransacRes = zeros(3, vars.nSaccades);
plotAng = zeros(3, vars.nSaccades);

% x-1, y-2, z-3 
for i=1:3
    for j=1:vars.nSaccades
        %% Simulate points
        R = eul2rotm(-angles(j)*[(3-i)==0 (2-i)==0 (1-i)==0]);
        T = (R-I)*vars.baseline;
        [Mw, M1, M2, m1, m2, err] = simulator(vars.nMatches, R, T, vars.maxDistToCam, vars.minDistToCam, vars.baseline, vars.intrinsics, vars.imgDim);
        if err == 1
            continue;
        end
        [m1, m2] = falseMatchesGen(m1, m2, vars.nFalseMatches, vars.imgDim);
        [m1, m2] = noiseGen(m1, m2, vars.nNoisePixels, vars.imgDim);
        if vars.ransac.on
            [m1, m2, err, ransacRes(i, j)] = ransacByProcrustes(m1, m2, vars.intrinsics, vars.projectionRadius, vars.minMatches, vars.maxMatches, vars.ransac);
            if err == 1
                continue;
            end
        end
        figure; plot(m1(1,:), m1(2,:), 'bx'); hold on; plot(m2(1,:), m2(2,:), 'rx');
        
        % Save angles for plot
        plotAng(i, j) = angles(j);
        
        %% Estimate transformation error
        [eRi, eTi]= estimator(m1, m2, vars.projectionRadius, vars.intrinsics, vars.baseline, rotm2eul(R), T, vars.methods);
        sizeeRi = size(eRi, 2);
        for n=1:sizeeRi
            eR(3*(n-1)+i, j) = eRi(n); 
            eT(3*(n-1)+i, j) = eTi(n); 
        end
        
        axisCount(i) = axisCount(i) +1;
    end  
end
 
end