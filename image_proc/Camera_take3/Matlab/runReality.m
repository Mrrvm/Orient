function [plotAng, axisCount, eRoppr, eRfpro, eRmbpe, eRepog] = runReality(filepath, samplePer, enoughPer, maxIters, maxErr, B,  radius, K)
%runReality Estimate transformation on real data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   filepath          Filepath of images 
%   samplePer      Percentage of the points set to target randomly
%   enoughPer     Percentage of the points set to accept the model
%   maxErr           Maximum error allowed for a good model
%   maxIters        Maximum number of iterations for ransac
%   B                    Baseline
%   radius            Sphere radius
%   K                    Intrinsics matrix
%   imgDir           Image directory to read from
% Output
%   plotAng          Angles used for plotting
%   axisCount      Number of images per axis
%   eR...               Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
axisCount = zeros(1,3);
dirs = dir(fullfile(filepath,'*'));
dirs = dirs(~ismember({dirs.name},{'.','..'}));

for z = 1:numel(dirs)

     if dirs(z).name == 'x'
            i = 1; 
            load(strcat(filepath, 'x/images.mat'));
            load(strcat(filepath, 'x/rotations.mat'));
            axisCount(1) = numel(rotations);
     end
     if dirs(z).name == 'y'
            i = 2; 
            load(strcat(filepath, 'y/images.mat'));
            load(strcat(filepath, 'y/rotations.mat'));
            axisCount(2) = numel(rotations);
     end
     if dirs(z).name == 'z'
            i = 3; 
            load(strcat(filepath, 'z/images.mat'));
            load(strcat(filepath, 'z/rotations.mat'));
            axisCount(3) = numel(rotations);
     end
    
     j = 1;

     for k = 1:numel(rotations)
        %% Extract matches from images
        img1 = images(rotations(k).indImg1).img;
        img2 = images(rotations(k).indImg2).img;
        p1 = detectSURFFeatures(img1);
        p2 = detectSURFFeatures(img2);
        [f1,vpts1] = extractFeatures(img1, p1);
        [f2,vpts2] = extractFeatures(img2, p2);
        matches = matchFeatures(f1, f2, 'Method', 'Exhaustive',  'MatchThreshold', 100, 'Unique', true);
        m1a = vpts1(matches(:,1));
        m2a = vpts2(matches(:,2));
        bestInds = ransacByProcrustes(m1a.Location', m2a.Location', K, radius, maxErr, maxIters, samplePer, enoughPer);
        m1Best = m1a(bestInds'>0,:);
        m2Best = m2a(bestInds'>0,:);
        m1 = double(m1Best.Location');
        m2 = double(m2Best.Location');
        %figure; showMatchedFeatures(img1, img2, m1Best', m2Best');
        
        if((size(m1, 2) <= 8) && (size(m2, 2) <= 8))
            continue;
        end
        
        % Save angles for plot
        plotAng(i,j) = rotations(k).angle;

        %% Estimate transformation error
        [eRoppr(i,j), eRfpro(i,j), eRmbpe(i,j), eRepog(i,j) ]= estimator(m1, m2, radius, K, B, rotations(k).rot);
        j = j + 1;

     end
    
    clear images rotations;
    
end

end

