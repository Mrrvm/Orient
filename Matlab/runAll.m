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
    angles = generateAngles(vars.nSaccades, vars.saccadeDistrSigma);
    iters = max(round((max(vars.baseline.max)-min(vars.baseline.min))/vars.baseline.inc));
    baselines = zeros(3, iters);
    baselines(:, 1) = vars.baseline.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters
        baselines(:, i+1) = baselines(:, i) + vars.baseline.inc;
        vars.baseline = baselines(:, i);
         [~, ~, eR, eT, ransacRes] =  runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean(safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :), 2), 1);
        end
        i = i + 1;
    end
    plotError(baselines(1:iters), meRAllAxis, 'Baseline (m)', 'Error per baseline  - Sim data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end


%==========================================================================
% Test error in terms of distance using simulated data
if strcmp(TYPE, 'SIM_DISTANCES')
    angles = generateAngles(vars.nSaccades, vars.saccadeDistrSigma);
    iters = round((vars.distance.max-vars.distance.min)/vars.distance.inc);
    distances = zeros(1, iters);
    distances(1) = vars.distance.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters 
        distances(i+1) = distances(i) + vars.distance.inc;
        vars.distance.min = distance(i);
        vars.distance.max =  distances(i+1);
         [~, ~, eR, eT, ransacRes] =  runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :), 2), 1);
        end
        i = i + 1;
    end
    plotError(distances(1:iters), meRAllAxis, 'Distance (m)', 'Error per distance  - Sim data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end

%==========================================================================
% Test error in terms of noise using simulated data
if strcmp(TYPE, 'SIM_NOISES')
    angles = generateAngles(vars.nSaccades, vars.saccadeDistrSigma);
    iters = round((vars.noise.max-vars.noise.min)/vars.noise.inc);
    noises = zeros(1, iters);
    noises(1) = vars.noise.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters
        vars.noise.min = noises(i);
        [~, ~, eR, eT, ransacRes] =  runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :), 2), 1);
        end
        noises(i+1) = noises(i) + vars.noise.inc;
        i = i + 1;
    end
    plotError(noises(1:iters), meRAllAxis, 'Noise (pixel)', 'Error per pixel noise  - Sim data', vars.saveDir, vars.methods, vars.colors);
end

%==========================================================================
% Test error in terms of axis using simulated data CHANGE
if strcmp(TYPE, 'SIM_AXISANGLES')
    angles = generateAngles(vars.nSaccades, vars.saccadeDistrSigma);
    ang = angles(1:vars.nSaccades)*180/pi;
    [ang, axisCount, eR, eT, ransacRes] =  runSimulation(angles, vars);
    
    fileID = fopen(strcat(vars.saveDir, vars.filename),'w');
    % Compute mean error and variance
    fprintf('\n\t | [X] Mean | Variance \t | [Y] Mean | Variance \t | [Z] Mean | Variance | \n');
    fprintf(fileID, 'Rotation Axizs \t & X \t\t & \t\t\t  & Y \t\t & \t\t\t & Z \t\t & \t\t\t \n ');
    fprintf(fileID, '\t\t\t\t & Mean \t & Variance \t & Mean \t & Variance \t & Mean \t & Variance \n');
    for j=1:nMethods
            meR(:, j) = safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :), 2);
            pveR(:, j)  = sum( (eR( (3*(j-1)+1):(3*(j-1)+3), :) - meR(:, j)).^2, 2);
            if axisCount(1)
                fprintf('%s \t | %f | %f |', vars.methods{j}, meR(1, j), pveR(1, j)/axisCount(1));
                fprintf(fileID, '%s \t | %f | %f |', vars.methods{j}, meR(1, j), pveR(1, j)/axisCount(1));
            end
            if axisCount(2)
                fprintf('\t | %f | %f |', meR(2, j), pveR(2, j)/axisCount(2));
                fprintf(fileID, '\t | %f | %f |', meR(2, j), pveR(2, j)/axisCount(2));
            end
            if axisCount(3)
                fprintf('\t | %f | %f |', meR(3, j), pveR(3, j)/axisCount(3));
                fprintf(fileID, '\t | %f | %f |', meR(3, j), pveR(3, j)/axisCount(3));
            end 
    end
    % Determine the method with least mean error
    [minerror,ind] = min(sum(meR, 1));
    fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    
    for j=1:nMethods
        xerr(j, :) = eR( (3*(j-1)+1), :);
        yerr(j, :) = eR( (3*(j-1)+2), :);
        zerr(j, :) = eR( (3*(j-1)+3), :);
    end
    % Visualise error in terms of axis angle    
    plotError(ang, xerr, 'Angles (degrees)', 'Error per angle (x axis)  - Sim data', vars.saveDir, vars.methods, vars.colors);
    plotError(ang, yerr, 'Angles (degrees)', 'Error per angle (y axis)  - Sim data', vars.saveDir, vars.methods, vars.colors);
    plotError(ang, zerr, 'Angles (degrees)', 'Error per angle (z axis)  - Sim data', vars.saveDir, vars.methods, vars.colors);
    
end

%==========================================================================
% Test error in terms of distance using real data 
if strcmp(TYPE, 'REAL_DISTANCES')
    [dirs, dists] = readDirs(vars.distance.inputDir);
    for i = 1:size(dirs,1)
        [~, ~, eR, eT, ransacRes] =  runReality(strcat(vars.distance.inputDir, dirs(i).name, '/'), vars);
        clear imgs1 imgs2 axisCount;
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :) ) );
        endransacRes
    end
    plotError(dists, meRAllAxis, 'Distance (m)', 'Error per distance  - Real data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end

%==========================================================================
% Test error in terms of angles using real data CHANGE
if strcmp(TYPE, 'REAL_AXISANGLES')
    [ang, axisCount, eR, eT, ransacRes] = runReality(vars);
    
    fileID = fopen(strcat(vars.saveDir, vars.filename),'w');
    % Compute mean error and variance
    fprintf('\n\t | [X] Mean | Variance \t | [Y] Mean | Variance \t | [Z] Mean | Variance | \n');
    fprintf(fileID, '\n\t\t | [X] Mean | Variance \t\t | [Y] Mean | Variance \t\t | [Z] Mean | Variance | \n');
    for j=1:nMethods
            meR(:, j) = safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :), 2);
            pveR(:, j)  = sum( (eR( (3*(j-1)+1):(3*(j-1)+3), :) - meR(:, j)).^2, 2);
            if axisCount(1)
                fprintf('%s \t | %f | %f |', vars.methods{j}, meR(1, j), pveR(1, j)/axisCount(1));
                fprintf(fileID, '%s \t | %f | %f |', vars.methods{j}, meR(1, j), pveR(1, j)/axisCount(1));
            end
            if axisCount(2)
                fprintf('\t | %f | %f |', meR(2, j), pveR(2, j)/axisCount(2));
                fprintf(fileID, '\t | %f | %f |', meR(2, j), pveR(2, j)/axisCount(2));
            end
            if axisCount(3)
                fprintf('\t | %f | %f |', meR(3, j), pveR(3, j)/axisCount(3));
                fprintf(fileID, '\t | %f | %f |', meR(3, j), pveR(3, j)/axisCount(3));
            end
    end
        
    % Determine the method with least mean error
    [minerror, ind] = min(sum(meR, 1));
    fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);

    for j=1:nMethods
        xerr(j, :) = eR( (3*(j-1)+1), :);
        yerr(j, :) = eR( (3*(j-1)+2), :);
        zerr(j, :) = eR( (3*(j-1)+3), :);
    end
    
    plotError(ang(1,:), xerr, 'Angles (degrees)', 'Error per angle (x axis) - Real data', vars.saveDir, vars.methods, vars.colors);
    plotError(ang(2,:), yerr, 'Angles (degrees)', 'Error per angle (y axis) - Real data', vars.saveDir, vars.methods, vars.colors);
    plotError(ang(3,:), zerr, 'ransacResAngles (degrees)', 'Error per angle (z axis) - Real data', vars.saveDir, vars.methods, vars.colors);
    end

end

fprintf('\nRansac detected a %f % of bad matches\n\n', safeMean( safeMean(ransacRes, 2), 1));
