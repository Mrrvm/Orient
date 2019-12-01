function [angles, nrots, eR, eT, ransacRes, entropy, results] = runReality(vars)
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
   
if(exist(strcat(vars.inputDir, 'data.mat'), 'file') && exist(strcat(vars.inputDir, 'rotations.mat'), 'file'))
    load(strcat(vars.inputDir, 'data.mat'));
    load(strcat(vars.inputDir, 'rotations.mat'));
else
    return;
end
  
angles = 0;
sections = zeros(3, 3);
entropy = zeros(2, 1);
ransacRes = zeros(1, numel(rotations));
cameraParams = cameraParameters('IntrinsicMatrix', vars.intrinsics', 'RadialDistortion', vars.radialDist, 'TangentialDistortion', vars.tanDist); 

j = 1;
for k = 1:numel(rotations)
    
    %% Extract matches from images
    img1 = undistortImage(data(rotations(k).indImg1).img, cameraParams);
    img2 = undistortImage(data(rotations(k).indImg2).img, cameraParams);
    p1 = detectSURFFeatures(img1, 'MetricThreshold', 100);
    p2 = detectSURFFeatures(img2, 'MetricThreshold', 100);
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
        
        else
            if size(matches, 1) > vars.maxMatches
                m1 = double(m1a.Location(1:vars.maxMatches, :)');
                m2 = double(m2a.Location(1:vars.maxMatches, :)');
            end
        end
        %figure; showMatchedFeatures(img1, img2, m1', m2');  

        % Save angles for plot
        axang = rotm2axang(eul2rotm(rotations(k).rot));
        angles(j) = axang(4)*180/pi;

        %% Estimate transformation error
        [eRi, eTi, rot] = estimator(m1, m2, vars.projectionRadius, vars.intrinsics, vars.currBaseline, rotations(k).rot, [0 0 0], vars.methods);
        eR(:, j) = eRi*180/pi; 
        eT(:, j) = eTi; 

        results(j).indImg1 = rotations(k).indImg1;
        results(j).indImg2 = rotations(k).indImg2;
        results(j).rot = rotations(k).rot*180/pi;
        results(j).angle = angles(j);
        results(j).oppr = rot(1, :)*180/pi;
        results(j).mbpe = rot(2, :)*180/pi;
        results(j).grat = rot(3, :)*180/pi;
        results(j).errd = eRi*180/pi;
        
        % Check image sections
        sections = whichImageSections(m1, vars.imgDim);
        a1 = findEntropy(sections);
        sections = whichImageSections(m2, vars.imgDim);
        a2 = findEntropy(sections);
        results(j).entropy = [a1 a2];

        j = j + 1;
    end

end

nrots = j;

end


