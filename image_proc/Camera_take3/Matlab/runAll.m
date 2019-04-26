close all;
clear;

%% Testing types (Choose which are ON ...)
DISTANCES        = 0;
NOISES              = 0;
ANGLES             = 1;

%% Constants
focalLength = 1;                                                 % intrinsic parameters
m2pix         = [3779.53 3779.53];                       % ...
skew           = 0;                                                  % ...
axisOffset   = [0 0];                                             % ...
K = [focalLength*m2pix(1) skew                           axisOffset(1); 
     0                                   focalLength*m2pix(2) axisOffset(2); 
     0                                   0                                 1           ]; 
nMatches    = 20;                                                % number of point matches
B                 = [0.0 0.0 0.07]';                              % baseline
nAngles      = 50;                                                % number of angles to test per axis
sigma         = 5;                                                  % normal distribution sigma
radius         = 5;                                                  % sphere radius
maxConstD = 5;                                                  % max distance to camera for angle and noise testing
minConstD  = 1;                                             % min distance to camera
nPixels        = 2;                                                  % number of pixels to deviate in noise
angles         = generateAngles(nAngles, sigma); % angles to test
%% Constants for DISTANCE (in m)
maxDistance = 15;
minDistance = 0.05;
incDistance = 2;                               
%% Constants for NOISE (in pixels)
maxNoise = 6;
minNoise = 1;
incNoise = 1;

%% Test error in terms of distance
if DISTANCES
    iters = round((maxDistance-minDistance)/incDistance);
    distances = zeros(1, iters);
    distances(1) = minDistance;
    meRAllAxis = zeros(4, iters);
    i = 1;
    while i <= iters
        distances(i+1) = distances(i) + incDistance;
        [eRoppr, eRfpro, eRmbpe, eRepog] =  runSimulation(angles, radius, K, nMatches, distances(i+1), distances(i), B, nAngles, nPixels);
        meRAllAxis(1,i) = mean(mean(eRoppr, 2));
        meRAllAxis(2,i) = mean(mean(eRfpro, 2));
        meRAllAxis(3,i) = mean(mean(eRmbpe, 2));
        meRAllAxis(4,i) = mean(mean(eRepog, 2));
        i = i + 1;
    end
    plotError(distances(1:iters), meRAllAxis, 'Distance (m)', 'Error per distance');
    clear meRAllAxis;
end

%% Test error in terms of noise
if NOISES
    iters = round((maxNoise-minNoise)/incNoise);
    noises = zeros(1, iters);
    noises(1) = minNoise;
    meRAllAxis = zeros(4, iters);
    i = 1;
    while i <= iters
        [eRoppr, eRfpro, eRmbpe, eRepog] =  runSimulation(angles, radius, K, nMatches, maxConstD, minConstD, B, nAngles, noises(i));
        meRAllAxis(1,i) = mean(mean(eRoppr, 2));
        meRAllAxis(2,i) = mean(mean(eRfpro, 2));
        meRAllAxis(3,i) = mean(mean(eRmbpe, 2));
        meRAllAxis(4,i) = mean(mean(eRepog, 2));
        noises(i+1) = noises(i) + incNoise;
        i = i + 1;
    end
    plotError(noises(1:iters), meRAllAxis, 'Noise (pixel)', 'Error per pixel noise');
    clear meRAllAxis;
end

%% Test error in terms of axis
if ANGLES
    [eRoppr, eRfpro, eRmbpe, eRepog] =  runSimulation(angles, radius, K, nMatches, maxConstD, minConstD, B, nAngles, nPixels);
    % Compute mean error and variance
    meRoppr = mean(eRoppr, 2);
    meRfpro = mean(eRfpro, 2);
    meRmbpe = mean(eRmbpe, 2);
    meRepog = mean(eRepog, 2);
    veRoppr = sum((eRoppr - meRoppr).^2,2)/nMatches;
    veRfpro = sum((eRfpro - meRfpro).^2,2)/nMatches;
    veRmbpe = sum((eRmbpe - meRmbpe).^2,2)/nMatches;
    veRepog = sum((eRepog - meRepog).^2,2)/nMatches;
    fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meRoppr(1), veRoppr(1), meRfpro(1), veRfpro(1), meRmbpe(1), veRmbpe(1), meRepog(1), veRepog(1));
    fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meRoppr(2), veRoppr(2), meRfpro(2), veRfpro(2), meRmbpe(2), veRmbpe(2), meRepog(2), veRepog(2));
    fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meRoppr(3), veRoppr(3), meRfpro(3), veRfpro(3), meRmbpe(3), veRmbpe(3), meRepog(3), veRepog(3));

    % Determine the method with least mean error
    meR = [meRoppr meRfpro meRmbpe meRepog];
    [minerror, method] = min(sum(meR));
    label = {'error oppr', 'error fpro', 'error mbpe', 'error epog'};
    fprintf('\nBest method was %s with %f half radians of error.\n\n', string(label(method)), minerror);

    % Visualise error in terms of axis angle
    ang = angles(1:nAngles)*180/pi;
    plotError(ang, [eRoppr(1,:); eRfpro(1,:); eRmbpe(1,:); eRepog(1,:)], 'Angles (degrees)', 'Error per angle (x axis)');
    plotError(ang, [eRoppr(2,:); eRfpro(2,:); eRmbpe(2,:); eRepog(2,:)], 'Angles (degrees)', 'Error per angle (y axis)');
    plotError(ang, [eRoppr(3,:); eRfpro(3,:); eRmbpe(3,:); eRepog(3,:)], 'Angles (degrees)', 'Error per angle (z axis)');
end


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



