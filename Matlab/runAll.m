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
    angles = generateAngles(vars.nSaccades, vars.saccadeSigma);
    iters = max(round((max(vars.baseline.max)-min(vars.baseline.min))/vars.baseline.inc));
    baselines = zeros(3, iters);
    baselines(:, 1) = vars.baseline.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters
        baselines(:, i+1) = baselines(:, i) + vars.baseline.inc;
        vars.currBaseline = baselines(:, i);
         [ang, axisCount, eR, eT, ransacRes] = runSimulation(angles, vars);
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
    angles = generateAngles(vars.nSaccades, vars.saccadeSigma);
    iters = round((vars.distToCam.max-vars.distToCam.min)/vars.distToCam.inc);
    distances = zeros(1, iters);
    distances(1) = vars.distToCam.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters 
        distances(i+1) = distances(i) + vars.distToCam.inc;
        vars.currDistToCam.min = distance(i);
        vars.currDistToCam.max = distance(i+1);
         [ang, axisCount, eR, eT, ransacRes] =  runSimulation(angles, vars);
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
    angles = generateAngles(vars.nSaccades, vars.saccadeSigma);
    iters = round((vars.noisePixelsSigma.max-vars.noisePixelsSigma.min)/vars.noisePixelsSigma.inc);
    noises = zeros(1, iters);
    noises(1) = vars.noisePixelsSigma.min;
    meRAllAxis = zeros(nMethods, iters);
    i = 1;
    while i <= iters
        vars.currNoisePixelsSigma = noises(i);
        [ang, axisCount, eR, eT, ransacRes] = runSimulation(angles, vars);
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :), 2), 1);
        end
        noises(i+1) = noises(i) + vars.noisePixelsSigma.inc;
        i = i + 1;
    end
    plotError(noises(1:iters), meRAllAxis, 'Noise (pixel)', 'Error per pixel noise  - Sim data', vars.saveDir, vars.methods, vars.colors);
end

%==========================================================================
% Test error in terms of axis using simulated data CHANGE
if strcmp(TYPE, 'SIM_AXISANGLES')
    angles = generateAngles(vars.nSaccades, vars.saccadeSigma);
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
            fprintf('\n');
            fprintf(fileID, '\n');
    end
    % Determine the method with least mean error
    [minerror,ind] = min(sum(meR, 1));
    fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
    
    for j=1:nMethods
        xerr(j, :) = eR( (3*(j-1)+1), (eR( (3*(j-1)+1), :) ~= 0 ));
        yerr(j, :) = eR( (3*(j-1)+2), (eR( (3*(j-1)+2), :) ~= 0));
        zerr(j, :) = eR( (3*(j-1)+3), (eR( (3*(j-1)+3), :) ~= 0));
    end
    % Visualise error in terms of axis angle    
    plotError(ang(1, ang(1,:) ~= 0), xerr, 'Angles (degrees)', 'Error per angle (x axis)  - Sim data', vars.saveDir, vars.methods, vars.colors);
    plotError(ang(2, ang(2,:) ~= 0), yerr, 'Angles (degrees)', 'Error per angle (y axis)  - Sim data', vars.saveDir, vars.methods, vars.colors);
    plotError(ang(3, ang(3,:) ~= 0), zerr, 'Angles (degrees)', 'Error per angle (z axis)  - Sim data', vars.saveDir, vars.methods, vars.colors);
    
end

%==========================================================================
% Test error in terms of distance using real data 
if strcmp(TYPE, 'REAL_DISTANCES')
    [dirs, dists] = readDirs(vars.distance.inputDir);
    for i = 1:size(dirs,1)
        [ang, axisCount, eR, eT, ransacRes] =  runReality(strcat(vars.distance.inputDir, dirs(i).name, '/'), vars);
        clear imgs1 imgs2 axisCount;
        for j=1:nMethods
            meRAllAxis(j, i) = safeMean( safeMean( eR( (3*(j-1)+1):(3*(j-1)+3), :) ) );
        end
    end
    plotError(dists, meRAllAxis, 'Distance (m)', 'Error per distance  - Real data', vars.saveDir, vars.methods, vars.colors);
    clear meRAllAxis;
end

%==========================================================================
% Test error in terms of angles using real data CHANGE
if strcmp(TYPE, 'REAL_AXISANGLES')
    [ang, axisCount, eR, eT, ransacRes] = runReality(vars);

    fileID = fopen(strcat(vars.saveDir, vars.filename),'w');
    
    if vars.peraxis == 0
        fprintf('\n\t | Mean | Variance \t');
        fprintf(fileID, '\n\t\t | Mean | Variance \t\t');
        for j=1:nMethods
             meR(j) = safeMean(eR(j, :), 2);
             pveR(j)  = sum( (eR(j, :) - meR(j)).^2, 2);
             fprintf('%s \t | %f | %f |', vars.methods{j}, meR(j), pveR(j)/axisCount);
             fprintf(fileID, '%s \t | %f | %f |', vars.methods{j}, meR(j), pveR(j)/axisCount);
             fprintf('\n');
             fprintf(fileID, '\n');
        end
        
        [minerror, ind] = min(meR);
        fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
        fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
        
        plotError(ang(1, :), eR, 'Angles (degrees)', 'Error per angle - Real data', vars.saveDir, vars.methods, vars.colors);
        
    else
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
                fprintf('\n');
                fprintf(fileID, '\n');
        end

        % Determine the method with least mean error
        [minerror, ind] = min(sum(meR, 1));
        fprintf('\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);
        fprintf(fileID, '\nBest method was %s with %f degrees of mean error.\n\n', vars.methods{ind}, minerror/3);

        for j=1:nMethods
            xerr(j, :) = eR( (3*(j-1)+1), (eR( (3*(j-1)+1), :) ~= 0 ));
            yerr(j, :) = eR( (3*(j-1)+2), (eR( (3*(j-1)+2), :) ~= 0));
            zerr(j, :) = eR( (3*(j-1)+3), (eR( (3*(j-1)+3), :) ~= 0));
        end

        plotError(ang(1, ang(1,:) ~= 0), xerr, 'Angles (degrees)', 'Error per angle (x axis) - Real data', vars.saveDir, vars.methods, vars.colors);
        plotError(ang(2, ang(2,:) ~= 0), yerr, 'Angles (degrees)', 'Error per angle (y axis) - Real data', vars.saveDir, vars.methods, vars.colors);
        plotError(ang(3, ang(3,:) ~= 0), zerr, 'ransacResAngles (degrees)', 'Error per angle (z axis) - Real data', vars.saveDir, vars.methods, vars.colors);

    end     
end

if vars.ransac.on
    fprintf('\nRansac detected a %f  percentage of bad matches\n\n', safeMean( safeMean(ransacRes, 2), 1));
end

end



