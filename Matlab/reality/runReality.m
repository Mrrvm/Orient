function [plotAng, axisCount, eR, eT, ransacRes] = runReality(vars)
%runReality Estimate transformation on real data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   filepath       Filepath of images 
%   samplePer      Percentage of the points set to target randomly
%   enoughPer     Percentage of the points set to accept the model
%   maxErr           Maximum error allowed for a good model
%   maxIters        Maximum number of iterations for ransac
%   B                    currBaseline
%   radius            Sphere radius
%   K                    Intrinsics matrix
%   imgDir           Image directory to read from
%   minPoints      Minimum number of points allowed
%   maxPoints     Maximum number of points allowed
% Output
%   plotAng          Angles used for plotting
%   axisCount      Number of images per axis
%   eR...               Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
cameraParams = cameraParameters('IntrinsicMatrix', vars.intrinsics', 'RadialDistortion', vars.radialDist, 'TangentialDistortion', vars.tanDist); 

if vars.peraxis == 1
    
    axisCount = zeros(1,3);
    dirs = dir(fullfile(vars.inputDir,'*'));
    dirs = dirs(~ismember({dirs.name},{'.','..'}));

    for z = 1:numel(dirs)
         if dirs(z).name == 'x'
                if(exist(strcat(vars.inputDir, 'x/data.mat'), 'file') && exist(strcat(vars.inputDir, 'x/rotations.mat'), 'file'))
                    i = 1; 
                    load(strcat(vars.inputDir, 'x/data.mat'));
                    load(strcat(vars.inputDir, 'x/rotations.mat'));
                    axisCount(1) = numel(rotations);
                end
         end
         if dirs(z).name == 'y'
                if(exist(strcat(vars.inputDir, 'y/data.mat'), 'file') && exist(strcat(vars.inputDir, 'y/rotations.mat'), 'file'))
                    i = 2; 
                    load(strcat(vars.inputDir, 'y/data.mat'));
                    load(strcat(vars.inputDir, 'y/rotations.mat'));
                    axisCount(2) = numel(rotations);
                end
         end
         if dirs(z).name == 'z'
                if(exist(strcat(vars.inputDir, 'z/data.mat'), 'file') && exist(strcat(vars.inputDir, 'z/rotations.mat'), 'file'))
                    i = 3; 
                    load(strcat(vars.inputDir, 'z/data.mat'));
                    load(strcat(vars.inputDir, 'z/rotations.mat'));
                    axisCount(3) = numel(rotations);
                end
         end

         j = 1;
         ransacRes = zeros(3, numel(rotations));
         for k = 1:numel(rotations)
            %% Extract matches from images
            img1 = undistortImage(data(rotations(k).indImg1).img, cameraParams);
            img2 = undistortImage(data(rotations(k).indImg2).img, cameraParams);
            p1 = detectSURFFeatures(img1);
            p2 = detectSURFFeatures(img2);
            [f1,vpts1] = extractFeatures(img1, p1);
            [f2,vpts2] = extractFeatures(img2, p2);
            matches = matchFeatures(f1, f2, 'Method', 'Exhaustive',  'MatchThreshold', 100, 'Unique', true);
            m1a = vpts1(matches(:,1));
            m2a = vpts2(matches(:,2));
            if vars.ransac.on
                [m1, m2, err, ransacRes(i, j)] = ransacByProcrustes(m1a.Location', m2a.Location', vars.intrinsics, vars.projectionRadius, vars.minMatches, vars.maxMatches, vars.ransac);
                if err == 1
                    continue;
                end
            end
            %figure; showMatchedFeatures(img1, img2, m1', m2');  

            % Save angles for plot
            plotAng(i,j) = rotations(k).angle;

            %% Estimate transformation error
            [eRi, eTi]= estimator(m1, m2, vars.projectionRadius, vars.intrinsics, vars.currBaseline, rotations(k).rot, rotations(k).tr, vars.methods);
            sizeeRi = size(eRi, 2);
            for n=1:sizeeRi
                eR(3*(n-1)+i, j) = eRi(n); 
                eT(3*(n-1)+i, j) = eTi(n); 
            end
            j = j + 1;

         end

        clear data rotations;

    end

else
    
    axisCount = 0;
    if(exist(strcat(vars.inputDir, 'data.mat'), 'file') && exist(strcat(vars.inputDir, 'rotations.mat'), 'file'))
        load(strcat(vars.inputDir, 'data.mat'));
        load(strcat(vars.inputDir, 'rotations.mat'));
    else
        return;
    end
    
    j = 1;
    ransacRes = zeros(numel(rotations));
    for k = 1:numel(rotations)
        %% Extract matches from images
        img1 = undistortImage(data(rotations(k).indImg1).img, cameraParams);
        img2 = undistortImage(data(rotations(k).indImg2).img, cameraParams);
        p1 = detectSURFFeatures(img1);
        p2 = detectSURFFeatures(img2);
        [f1,vpts1] = extractFeatures(img1, p1);
        [f2,vpts2] = extractFeatures(img2, p2);
        matches = matchFeatures(f1, f2, 'Method', 'Exhaustive',  'MatchThreshold', 100, 'Unique', true);
        if size(matches, 1) > 3
            m1a = vpts1(matches(:,1));
            m2a = vpts2(matches(:,2));
            if vars.ransac.on
                [m1, m2, err, ransacRes(j)] = ransacByProcrustes(m1a.Location', m2a.Location', vars.intrinsics, vars.projectionRadius, vars.minMatches, vars.maxMatches, vars.ransac);
                if err == 1
                    continue;
                end
            end
            %figure; showMatchedFeatures(img1, img2, m1', m2');  

            % Save angles for plot
            plotAng(:, j) = rotations(k).rot*180/pi;

            %% Estimate transformation error
            [eRi, eTi] = estimator(m1, m2, vars.projectionRadius, vars.intrinsics, vars.currBaseline, rotations(k).rot, [0 0 0], vars.methods);
            eR(:, j) = eRi; 
            eT(:, j) = eTi; 
            j = j + 1;
        end

    end
    axisCount = j;
end

end

