function runAll(TYPE, vars)

%% Testing types (Choose one ...)
% SIM_BASELINES
% SIM_DISTANCES                     
% SIM_NOISES                           
% SIM_AXISANGLES                   
% REAL_DISTANCES                  
% REAL_AXISANGLES    

%% Test error in terms of the baseline size using simulated data
if strcmp(TYPE, 'SIM_BASELINES')
    angles = generateAngles(vars.nAngles, vars.sigma);
    iters = max(round((max(vars.baseline.max)-min(vars.baseline.min))/vars.baseline.inc));
    baselines = zeros(3, iters);
    baselines(:, 1) = vars.baseline.min;
    meRAllAxis = zeros(4, iters);
    i = 1;
    while i <= iters
        baselines(:, i+1) = baselines(:, i) + vars.baseline.inc;
        [eRoppr, eRmee, eRmbpe, eRn8p] =  runSimulation(angles, vars.radius, vars.intrinsics, vars.nMatches, vars.maxD, vars.minD, baselines(:, i), vars.nAngles, vars.nPixels, vars.imgDim);
        meRAllAxis(1,i) = mean(mean(eRoppr, 2));
        meRAllAxis(2,i) = mean(mean(eRmee, 2));
        meRAllAxis(3,i) = mean(mean(eRmbpe, 2));
        meRAllAxis(4,i) = mean(mean(eRn8p, 2));
        i = i + 1;
    end
    plotError(baselines(1:iters), meRAllAxis, 'Baseline (m)', 'Error per baseline  - Sim data', vars.saveDir);
    clear meRAllAxis;
end

%% Test error in terms of distance using simulated data
if strcmp(TYPE, 'SIM_DISTANCES')
    angles = generateAngles(vars.nAngles, vars.sigma);
    iters = round((vars.distance.max-vars.distance.min)/vars.distance.inc);
    distances = zeros(1, iters);
    distances(1) = vars.distance.min;
    meRAllAxis = zeros(4, iters);
    i = 1;
    while i <= iters
        distances(i+1) = distances(i) + vars.distance.inc;
        [eRoppr, eRmee, eRmbpe, eRn8p] =  runSimulation(angles, vars.radius, vars.intrinsics, vars.nMatches, distances(i), distances(i), vars.baseline, vars.nAngles, vars.nPixels, vars.imgDim);
        meRAllAxis(1,i) = mean(mean(eRoppr, 2));
        meRAllAxis(2,i) = mean(mean(eRmee, 2));
        meRAllAxis(3,i) = mean(mean(eRmbpe, 2));
        meRAllAxis(4,i) = mean(mean(eRn8p, 2));
        i = i + 1;
    end
    plotError(distances(1:iters), meRAllAxis, 'Distance (m)', 'Error per distance  - Sim data', vars.saveDir);
    clear meRAllAxis;
end

%% Test error in terms of noise using simulated data
if strcmp(TYPE, 'SIM_NOISES')
    angles = generateAngles(vars.nAngles, vars.sigma);
    iters = round((vars.noise.max-vars.noise.min)/vars.noise.inc);
    noises = zeros(1, iters);
    noises(1) = vars.noise.min;
    meRAllAxis = zeros(4, iters);
    i = 1;
    while i <= iters
        [eRoppr, eRmee, eRmbpe, eRn8p] =  runSimulation(angles, vars.radius, vars.intrinsics, vars.nMatches, vars.maxD, vars.minD, vars.baseline, vars.nAngles, noises(i), vars.imgDim);
        meRAllAxis(1,i) = mean(mean(eRoppr, 2));
        meRAllAxis(2,i) = mean(mean(eRmee, 2));
        meRAllAxis(3,i) = mean(mean(eRmbpe, 2));
        meRAllAxis(4,i) = mean(mean(eRn8p, 2));
        noises(i+1) = noises(i) + vars.noise.inc;
        i = i + 1;
    end
    plotError(noises(1:iters), meRAllAxis, 'Noise (pixel)', 'Error per pixel noise  - Sim data', vars.saveDir);
end

%% Test error in terms of axis using simulated data
if strcmp(TYPE, 'SIM_AXISANGLES')
    angles = generateAngles(vars.nAngles, vars.sigma);
    [eRoppr, eRmee, eRmbpe, eRn8p] =  runSimulation(angles, vars.radius, vars.intrinsics, vars.nMatches, vars.maxD, vars.minD, vars.baseline, vars.nAngles, vars.nPixels, vars.imgDim);
    % Compute mean error and variance
    meRoppr = mean(eRoppr, 2);
    meRmee = mean(eRmee, 2);
    meRmbpe = mean(eRmbpe, 2);
    meRn8p = mean(eRn8p, 2);
    veRoppr = sum((eRoppr - meRoppr).^2,2)/vars.nAngles;
    veRmee = sum((eRmee - meRmee).^2,2)/vars.nAngles;
    veRmbpe = sum((eRmbpe - meRmbpe).^2,2)/vars.nAngles;
    veRn8p = sum((eRn8p - meRn8p).^2,2)/vars.nAngles;
    fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n MEE | %f | %f \n MBPE | %f | %f \n N8P | %f | %f \n\n', meRoppr(1), veRoppr(1), meRmee(1), veRmee(1), meRmbpe(1), veRmbpe(1), meRn8p(1), veRn8p(1));
    fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n MEE | %f | %f \n MBPE | %f | %f \n N8P | %f | %f \n\n', meRoppr(2), veRoppr(2), meRmee(2), veRmee(2), meRmbpe(2), veRmbpe(2), meRn8p(2), veRn8p(2));
    fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n MEE | %f | %f \n MBPE | %f | %f \n N8P | %f | %f \n\n', meRoppr(3), veRoppr(3), meRmee(3), veRmee(3), meRmbpe(3), veRmbpe(3), meRn8p(3), veRn8p(3));

    % Determine the method with least mean error
    meR = [meRoppr meRmee meRmbpe meRn8p];
    [minerror, method] = min(sum(meR));
    label = {'error oppr', 'error mee', 'error mbpe', 'error n8p'};
    fprintf('\nBest method was %s with %f degrees of error.\n\n', string(label(method)), minerror);

    % Visualise error in terms of axis angle
    ang = angles(1:vars.nAngles)*180/pi;
    plotError(ang, [eRoppr(1,:); eRmee(1,:); eRmbpe(1,:); eRn8p(1,:)], 'Angles (degrees)', 'Error per angle (x axis)  - Sim data', vars.saveDir);
    plotError(ang, [eRoppr(2,:); eRmee(2,:); eRmbpe(2,:); eRn8p(2,:)], 'Angles (degrees)', 'Error per angle (y axis)  - Sim data', vars.saveDir);
    plotError(ang, [eRoppr(3,:); eRmee(3,:); eRmbpe(3,:); eRn8p(3,:)], 'Angles (degrees)', 'Error per angle (z axis)  - Sim data', vars.saveDir);
end


%% Test error in terms of distance using real data
if strcmp(TYPE, 'REAL_DISTANCES')
    [dirs, dists] = readDirs(vars.distance.inputDir);
    for i = 1:size(dirs,1)
        [~, ~, eRoppr, eRmee, eRmbpe, eRn8p] =  runReality(strcat(vars.distance.inputDir, dirs(i).name, '/'), vars.ransac.samplePer, vars.ransac.enoughPer, vars.ransac.maxIters, vars.ransac.maxErr, vars.baseline, vars.radius, vars.intrinsics, vars.minMatches, vars.maxMatches, vars.radialDist, vars.tanDist);
        clear imgs1 imgs2 axisCount;
        meRAllAxis(1,i) = safeMean(safeMean(eRoppr, 2), 1);
        meRAllAxis(2,i) = safeMean(safeMean(eRmee, 2), 1);
        meRAllAxis(3,i) = safeMean(safeMean(eRmbpe, 2), 1);
        meRAllAxis(4,i) = safeMean(safeMean(eRn8p, 2), 1);
    end
    plotError(dists, meRAllAxis, 'Distance (m)', 'Error per distance  - Real data', vars.saveDir);
    clear meRAllAxis;
end

%% Test error in terms of angles using real data 
if strcmp(TYPE, 'REAL_AXISANGLES')
    [ang, axisCount, eRoppr, eRmee, eRmbpe, eRn8p] = runReality(vars.inputDir, vars.ransac.samplePer, vars.ransac.enoughPer, vars.ransac.maxIters, vars.ransac.maxErr, vars.baseline, vars.radius, vars.intrinsics, vars.minMatches, vars.maxMatches, vars.radialDist, vars.tanDist);
    % Compute mean error and variance
    meRoppr = safeMean(eRoppr, 2);
    meRmee = safeMean(eRmee, 2);
    meRmbpe = safeMean(eRmbpe, 2);
    meRn8p = safeMean(eRn8p, 2);
    pveRoppr = sum((eRoppr - meRoppr).^2,2);
    pveRmee = sum((eRmee - meRmee).^2,2);
    pveRmbpe = sum((eRmbpe - meRmbpe).^2,2);
    pveRn8p = sum((eRn8p - meRn8p).^2,2);
    % Visualise error in terms of axis angle
    if axisCount(1)
        fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n MEE | %f | %f \n MBPE | %f | %f \n N8P | %f | %f \n\n', meRoppr(1), pveRoppr(1)/axisCount(1), meRmee(1), pveRmee(1)/axisCount(1), meRmbpe(1), pveRmbpe(1)/axisCount(1), meRn8p(1), pveRn8p(1)/axisCount(1));
        plotError(ang(1,:), [eRoppr(1,:); eRmee(1,:); eRmbpe(1,:); eRn8p(1,:)], 'Angles (degrees)', 'Error per angle (x axis) - Real data', vars.saveDir);
    end
    if axisCount(2)
        fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n MEE | %f | %f \n MBPE | %f | %f \n N8P | %f | %f \n\n', meRoppr(2), pveRoppr(2)/axisCount(2), meRmee(2), pveRmee(2)/axisCount(2), meRmbpe(2), pveRmbpe(2)/axisCount(2), meRn8p(2), pveRn8p(2)/axisCount(2));
        plotError(ang(2,:), [eRoppr(2,:); eRmee(2,:); eRmbpe(2,:); eRn8p(2,:)], 'Angles (degrees)', 'Error per angle (y axis) - Real data', vars.saveDir);
    end
    if axisCount(3)
        fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n MEE | %f | %f \n MBPE | %f | %f \n N8P | %f | %f \n\n', meRoppr(3), pveRoppr(3)/axisCount(3), meRmee(3), pveRmee(3)/axisCount(3), meRmbpe(3), pveRmbpe(3)/axisCount(3), meRn8p(3), pveRn8p(3)/axisCount(3));
        plotError(ang(3,:), [eRoppr(3,:); eRmee(3,:); eRmbpe(3,:); eRn8p(3,:)], 'Angles (degrees)', 'Error per angle (z axis) - Real data', vars.saveDir);
    end
    % Determine the method with least mean error
    meR = [meRoppr meRmee meRmbpe meRn8p];
    [minerror, method] = min(sum(meR, 1));
    label = {'error oppr', 'error mee', 'error mbpe', 'error n8p'};
    fprintf('\nBest method was %s with %f degrees of error.\n\n', string(label(method)), minerror);

end

end