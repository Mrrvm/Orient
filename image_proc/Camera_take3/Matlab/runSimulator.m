close all;
clear;

%% Testing types (Choose which are ON ...)
DISTANCES        = 0;
NOISES           = 0;
ANGLES           = 1;

%% Constants
focalLength = 1;                            % Intrinsic parameters
m2pix = [3779.5275591 3779.5275591];        % ...
skew = 0;                                   % ...
axisOffset = [0 0];                         % ...
K = [focalLength*m2pix(1) skew                 axisOffset(1); 
     0                    focalLength*m2pix(2) axisOffset(2); 
     0                    0                    1           ]; 
nMatches = 20;                              % number of point matches
B = [0.0 0.0 0.07]';                        % baseline
nAngles = 10;                               % number of angles to test per axis
sigma = 5;                                  % normal distribution sigma
radius = 1;                                 % sphere radius
angles = generateAngles(nAngles, sigma);    % angles to test
maxConstD = 0.5;                            % max distance to camera for angle and noise testing
minConstD = 0.05;                           % min distance to camera
nPixels = 2;                                % number of pixels to deviate in noise
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
        [eRoppr, eRfpro, eRmbpe, eRepog] = simulator(angles, radius, K, nMatches, distances(i+1), distances(i), B, nAngles, nPixels);
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
        [eRoppr, eRfpro, eRmbpe, eRepog] = simulator(angles, radius, K, nMatches, maxConstD, minConstD, B, nAngles, noises(i));
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
    [eRoppr, eRfpro, eRmbpe, eRepog] = simulator(angles, radius, K, nMatches, maxConstD, minConstD, B, nAngles, nPixels);
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






