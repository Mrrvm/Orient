function runAll(TYPE, vars)

%% Testing types (Choose one ...)
% SIM_BASELINES
% SIM_DISTANCES                     
% SIM_NOISES                           
% SIM_AXISANGLES                   
% REAL_DISTANCES                  
% REAL_AXISANGLES    

nMethods = size(vars.methods, 2);

%==========================================================================
% Test error in terms of the baseline size using simulated data
if strcmp(TYPE, 'SIM_BASELINES')
    angles = zeros(vars.nSaccades, 3);
    iters = max(round((max(vars.baseline.max)-min(vars.baseline.min))/vars.baseline.inc));
    baselines = zeros(3, iters);
        
    if vars.axis == 'x'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        bi = 1;
        baselines(bi, 1) = vars.baseline.min;
    end
    if vars.axis == 'y'
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        bi = 2;
        baselines(bi, 1) = vars.baseline.min;
    end
    if vars.axis == 'z'
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        bi = 3;
        baselines(bi, 1) = vars.baseline.min;
    end
    if vars.axis == 'all'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        baselines(:, 1) = vars.baseline.min;
    end
    i = 1;
    meRAllAxis = zeros(nMethods, iters);

    while i <= iters
        if vars.axis == 'all'
            baselines(:, i+1) = baselines(:, i) + vars.baseline.inc;
            vars.currBaseline = baselines(:, i);
        else
            baselines(bi, i+1) = baselines(bi, i) + vars.baseline.inc;
            vars.currBaseline = baselines(:, i);
        end
        [ang, nrots, eR, eT, ransacRes] = runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean(safeMean( eR(j, :), 2), 1);
        end
        i = i + 1;
    end
    plotError(baselines(1:iters), meRAllAxis, 'Baseline (m)', 'Error per baseline  - Sim data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end


%==========================================================================
% Test error in terms of distance using simulated data
if strcmp(TYPE, 'SIM_DISTANCES')
    angles = zeros(vars.nSaccades, 3);
    if vars.axis == 'x'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'y'
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'z'
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'all'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    iters = round((vars.distToCam.max-vars.distToCam.min)/vars.distToCam.inc);
    distances = zeros(1, iters);
    distances(1) = vars.distToCam.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters 
        distances(i+1) = distances(i) + vars.distToCam.inc;
        vars.currDistToCam.min = distances(i);
        vars.currDistToCam.max = distances(i+1);
         [ang, nrots, eR, eT, ransacRes] =  runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR(j, :), 2), 1);
        end
        i = i + 1;
    end
    plotError(distances(1:iters), meRAllAxis, 'Distance (m)', 'Error per depth  - Sim data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end

%==========================================================================
% Test error in terms of noise using simulated data
if strcmp(TYPE, 'SIM_NOISES')
    angles = zeros(vars.nSaccades, 3);
    if vars.axis == 'x'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'y'
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'z'
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'all'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    iters = round((vars.noisePixelsSigma.max-vars.noisePixelsSigma.min)/vars.noisePixelsSigma.inc);
    noises = zeros(1, iters);
    noises(1) = vars.noisePixelsSigma.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters
        vars.currNoisePixelsSigma = noises(i);
        [ang, nrots, eR, eT, ransacRes] = runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR(j, :), 2), 1);
        end
        noises(i+1) = noises(i) + vars.noisePixelsSigma.inc;
        i = i + 1;
    end
    plotError(noises(1:iters), meRAllAxis, 'Noise (pixel)', 'Error per pixel noise  - Sim data', vars.saveDir, vars.methods, vars.colors);
end

%==========================================================================
% Test error in terms of axis using simulated data CHANGE
if strcmp(TYPE, 'SIM_AXISANGLES')
    angles = zeros(vars.nSaccades, 3);
    if vars.axis == 'x'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'y'
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'z'
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    if vars.axis == 'all'
        angles(:, 3) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 2) = generateAngles(vars.nSaccades, vars.saccadeSigma);
        angles(:, 1) = generateAngles(vars.nSaccades, vars.saccadeSigma);
    end
    [ang, nrots, eR, eT, ransacRes] =  runSimulation(angles, vars);
    
    fileID = fopen(strcat(vars.saveDir, vars.filename),'w');
    fprintf('\n\t\t | Mean | Variance \n');
    fprintf(fileID, '\n\t\t | Mean | Variance \n');
    for j=1:nMethods
            meR(j) = safeMean( eR(j, :), 2);
            pveR(j)  = sum( (eR(j, :) - meR(j)).^2, 2);
            if nrots
                fprintf('%s \t | %f | %f |', vars.methods{j}, meR(j), sqrt(pveR(j)/nrots));
                fprintf(fileID, '%s \t | %f | %f |', vars.methods{j}, meR(j), sqrt(pveR(j)/nrots));
            end
            fprintf('\n');
            fprintf(fileID, '\n');
    end
    % Determine the method with least mean error
    [minerror,ind] = min(meR);
    fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    
    for j=1:nMethods
        err(j, :) = eR(j, (eR(j, :) ~= 0 ));
    end
    % Visualise error in terms of axis angle    
    plotError(ang, err, 'Angles (degrees)', 'Error per angle  - Sim data', vars.saveDir, vars.methods, vars.colors);
end

%==========================================================================
% Test error in terms of distance using real data 
if strcmp(TYPE, 'REAL_DISTANCES')
    [dirs, dists] = readDirs(vars.distance.inputDir);
    for i = 1:size(dirs,1)
        [ang, axisCount, eR, eT, ransacRes] =  runReality(strcat(vars.distance.inputDir, dirs(i).name, '/'), vars);
        clear imgs1 imgs2 axisCount;
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR(j, :) ) );
        end
    end
    plotError(dists, meRAllAxis, 'Distance (m)', 'Error per distance  - Real data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end

%==========================================================================
% Test error in terms of angles using real data CHANGE
if strcmp(TYPE, 'REAL_AXISANGLES')
    [angles, nrots, eR, eT, ransacRes, entropy, results] = runReality(vars);

    fileID = fopen(strcat(vars.saveDir, vars.filename),'w');

    fprintf('\n\t\t | Mean | Variance \n');
    fprintf(fileID, '\n\t\t | Mean | Variance \n');

    hist = zeros(nMethods, 6);
    bins = [0.5 1 3 5 10];
    
    for j=1:nMethods
         hist(j, :) = createHistogram(eR(j, :), bins);
         meR(j) = safeMean(eR(j, :), 2);
         pveR(j)  = sum( (eR(j, :) - meR(j)).^2, 2);
         fprintf('%s \t | %f | %f |', vars.methods{j}, meR(j), sqrt(pveR(j)/nrots));
         fprintf(fileID, '%s \t | %f | %f |', vars.methods{j}, meR(j), sqrt(pveR(j)/nrots));
         fprintf('\n');
         fprintf(fileID, '\n');
    end    

    [minerror, ind] = min(meR);
    fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror);
    fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror);

    plotError(angles(1, :), eR, 'Angles (degrees)', 'Error per angle - Real data', vars.saveDir, vars.methods, vars.colors);
    save(strcat(vars.saveDir, '/results.mat') , 'results');
    hist
end

%==========================================================================
if vars.ransac.on
    fprintf('\nRansac detected a %f  percentage of bad matches\n\n', safeMean( safeMean(ransacRes, 2)*100, 1));
end

end



