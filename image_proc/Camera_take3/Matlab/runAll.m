close all;
clear;

%% Testing types (Choose one ...)
SIM_DISTANCES     = 0;
SIM_NOISES           = 0;
SIM_AXISANGLES   = 0;
REAL_DISTANCES   = 1;
REAL_AXISANGLES = 0;
% ATTENTION -----------------------------------------------
% Images must have the following name "im[1/2][x/y/z][angle in
% degrees].jpg" and be organized by distance in different directories.
% Each directory should be named d[distance in meters].
% ---------------------------------------------------------

%% Constants
if SIM_DISTANCES || SIM_NOISES || SIM_AXISANGLES
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
    minConstD  = 1;                                                  % min distance to camera
    nPixels        = 2;                                                  % number of pixels to deviate in noise
    angles         = generateAngles(nAngles, sigma); % angles to test
    %% Constants for DISTANCE (in m)
    if SIM_DISTANCES
        maxDistance = 15;
        minDistance  = 0.05;
        incDistance    = 2;      
    end
    %% Constants for NOISE (in pixels)
    if SIM_NOISE
        maxNoise = 6;
        minNoise  = 1;
        incNoise   = 1;
    end
else
    allFilesDir = '../cam_img';     % files directory
    imgsDir = 'd5';                      % imgs directory
    samplePer = 0.10;                 % ransac parameters
    enoughPer = 0.50;                % ...
    maxIters    = 20;                   % ...
    maxErr       = 0.0003;            % ...
    B                = [0.0 0.0 0.07]';  % baseline
    radius        = 1;
    K = [1.1446e+03 0                    9.8904e+02; 
            0                  1.1452e+03  7.554e+02  ;
            0                  0                    1                 ];
end

%% Test error in terms of distance using simulated data
if SIM_DISTANCES
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
    plotError(distances(1:iters), meRAllAxis, 'Distance (m)', 'Error per distance  - Sim data');
    clear meRAllAxis;
end

%% Test error in terms of noise using simulated data
if SIM_NOISES
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
    plotError(noises(1:iters), meRAllAxis, 'Noise (pixel)', 'Error per pixel noise  - Sim data');
    clear meRAllAxis;
end

%% Test error in terms of axis using simulated data
if SIM_AXISANGLES
    [eRoppr, eRfpro, eRmbpe, eRepog] =  runSimulation(angles, radius, K, nMatches, maxConstD, minConstD, B, nAngles, nPixels);
    % Compute mean error and variance
    meRoppr = mean(eRoppr, 2);
    meRfpro = mean(eRfpro, 2);
    meRmbpe = mean(eRmbpe, 2);
    meRepog = mean(eRepog, 2);
    veRoppr = sum((eRoppr - meRoppr).^2,2)/nAngles;
    veRfpro = sum((eRfpro - meRfpro).^2,2)/nAngles;
    veRmbpe = sum((eRmbpe - meRmbpe).^2,2)/nAngles;
    veRepog = sum((eRepog - meRepog).^2,2)/nAngles;
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
    plotError(ang, [eRoppr(1,:); eRfpro(1,:); eRmbpe(1,:); eRepog(1,:)], 'Angles (degrees)', 'Error per angle (x axis)  - Sim data');
    plotError(ang, [eRoppr(2,:); eRfpro(2,:); eRmbpe(2,:); eRepog(2,:)], 'Angles (degrees)', 'Error per angle (y axis)  - Sim data');
    plotError(ang, [eRoppr(3,:); eRfpro(3,:); eRmbpe(3,:); eRepog(3,:)], 'Angles (degrees)', 'Error per angle (z axis)  - Sim data');
end


%% Test error in terms of distance using real data
if REAL_DISTANCES
    [dirs, dists] = readDirs(allFilesDir);
    for i = 1:size(dirs,1)
        [imgs1, imgs2, axisCount] = readImages(strcat(allFilesDir, '/', dirs(i).name));
        [eRoppr, eRfpro, eRmbpe, eRepog] =  runReality(imgs1, imgs2, axisCount, samplePer, enoughPer, maxIters, maxErr, B,  radius, K);
        clear imgs1 imgs2 axisCount;
        meRAllAxis(1,i) = safeMean(safeMean(eRoppr, 2), 1);
        meRAllAxis(2,i) = safeMean(safeMean(eRfpro, 2), 1);
        meRAllAxis(3,i) = safeMean(safeMean(eRmbpe, 2), 1);
        meRAllAxis(4,i) = safeMean(safeMean(eRepog, 2), 1);
    end
    plotError(dists, meRAllAxis, 'Distance (m)', 'Error per distance  - Real data');
    clear meRAllAxis;
end

%% Test error in terms of angles using real data
if REAL_AXISANGLES
    [imgs1, imgs2, axisCount] = readImages(strcat(allFilesDir, '/', imgsDir));
    [angles, axisCount, eRoppr, eRfpro, eRmbpe, eRepog] = runReality(imgs1, imgs2, axisCount, samplePer, enoughPer, maxIters, maxErr, B,  radius, K);
    % Compute mean error and variance
    meRoppr = mean(eRoppr, 2);
    meRfpro = mean(eRfpro, 2);
    meRmbpe = mean(eRmbpe, 2);
    meRepog = mean(eRepog, 2);
    nAngles = size(eRoppr, 2);
    veRoppr = sum((eRoppr - meRoppr).^2,2)/nAngles;
    veRfpro = sum((eRfpro - meRfpro).^2,2)/nAngles;
    veRmbpe = sum((eRmbpe - meRmbpe).^2,2)/nAngles;
    veRepog = sum((eRepog - meRepog).^2,2)/nAngles;
    % Visualise error in terms of axis angle
    ang = angles*180/pi;
    if axisCount(1)
        fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meRoppr(1), veRoppr(1), meRfpro(1), veRfpro(1), meRmbpe(1), veRmbpe(1), meRepog(1), veRepog(1));
        plotError(ang(1,:), [eRoppr(1,:); eRfpro(1,:); eRmbpe(1,:); eRepog(1,:)], 'Angles (degrees)', 'Error per angle (x axis) - Real data');
    end
    if axisCount(2)
        fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meRoppr(2), veRoppr(2), meRfpro(2), veRfpro(2), meRmbpe(2), veRmbpe(2), meRepog(2), veRepog(2));
        plotError(ang(2,:), [eRoppr(2,:); eRfpro(2,:); eRmbpe(2,:); eRepog(2,:)], 'Angles (degrees)', 'Error per angle (y axis) - Real data');
    end
    if axisCount(3)
        fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meRoppr(3), veRoppr(3), meRfpro(3), veRfpro(3), meRmbpe(3), veRmbpe(3), meRepog(3), veRepog(3));
        plotError(ang(3,:), [eRoppr(3,:); eRfpro(3,:); eRmbpe(3,:); eRepog(3,:)], 'Angles (degrees)', 'Error per angle (z axis) - Real data');
    end
    % Determine the method with least mean error
    meR = [meRoppr meRfpro meRmbpe meRepog];
    [minerror, method] = min(sum(meR));
    label = {'error oppr', 'error fpro', 'error mbpe', 'error epog'};
    fprintf('\nBest method was %s with %f half radians of error.\n\n', string(label(method)), minerror);

end

