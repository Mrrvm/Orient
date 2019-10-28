function [ang, nrots, eR, eT, ransacRes] = runSimulation(angles, vars)
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
nrots = 0;
ransacRes = zeros(vars.nSaccades, 1);
ang = 0;
eR = zeros(size(vars.methods,2), 1);
eT = zeros(size(vars.methods,2), 1);

for j=1:vars.nSaccades
        
    % To simulate the camera rotation 30 deg
    camrot = angles(j, :); 
    % we need the points to rotate -30 deg 
    ptsrot = -angles(j, :);

    %% Simulate points
    R = eul2rotm(ptsrot);
    T = (R-I)*vars.currBaseline;
    [Mw, M1, M2, m1, m2, err] = simulator(vars.nMatches, R, T, vars.currDistToCam.max, vars.currDistToCam.min, vars.currBaseline, vars.intrinsics, vars.imgDim);
    if err == 1
        continue;
    end
    [m1, m2] = falseMatchesGen(m1, m2, vars.nFalseMatches, vars.imgDim);
    [m1, m2] = noiseGen(m1, m2, vars.currNoisePixelsSigma, vars.imgDim);
    if vars.ransac.on
        [m1, m2, err, ransacRes(j)] = ransacByProcrustes(m1, m2, vars.intrinsics, vars.projectionRadius, vars.minMatches, vars.maxMatches, vars.ransac);
        if err == 1
            continue;
        end
    end
    %figure; plot(m1(1,:), m1(2,:), 'bx'); hold on; plot(m2(1,:), m2(2,:), 'rx');

    nrots = nrots + 1;
    % Save angles for plot
    axang = rotm2axang(eul2rotm(angles(j,:)));
    ang(nrots) = axang(4)*180/pi;

    %% Estimate transformation error
    [eRi, eTi, ~]= estimator(m1, m2, vars.projectionRadius, vars.intrinsics, vars.currBaseline, camrot, T, vars.methods);
    eR(:, j) = eRi*180/pi; 
    eT(:, j) = eTi; 
 
end  

end