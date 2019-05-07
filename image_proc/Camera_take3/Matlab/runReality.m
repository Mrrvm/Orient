function [plotAng, axisCount, eRoppr, eRfpro, eRmbpe, eRepog] = runReality(imgs1, imgs2, axisCount, samplePer, enoughPer, maxIters, maxErr, B,  radius, K)
%runReality Estimate transformation on real data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   imgs1,imgs2 Data with grayscale images, axis, angle and 
%                         distance information
%   axisCount      Number of images per axis
%   samplePer      Percentage of the points set to target randomly
%   enoughPer     Percentage of the points set to accept the model
%   maxErr           Maximum error allowed for a good model
%   maxIters        Maximum number of iterations for ransac
%   B                    Baseline
%   radius            Sphere radius
%   K                    Intrinsics matrix
%   imgDir           Image directory to read from
% Output
%   eR...               Error from each method
%   plotAng          Angles used for plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
i = 1;
j = 1;
for k = 1:numel(imgs1)
    %% Extract matches from images
    img1 = imgs1(k).data;
    img2 = imgs2(k).data;
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
    figure; showMatchedFeatures(img1, img2, m1Best', m2Best');
        
    if k == axisCount(1) + 1
        i = 2; % change to y
        j = 1;
    end
    if k == axisCount(1) + axisCount(2) + 1
            i = 3; % change to z
            j = 1;
    end
    
    if i == 1
        angles = [imgs1(k).angle*pi/180 0 0];
    else
        if i == 2
            angles = [0 imgs1(k).angle*pi/180 0];
        else 
            if i ==3
                angles = [0 0 imgs1(k).angle*pi/180];
            end
        end
    end
    
    % Save angles for plot
    plotAng(i,j) = imgs1(k).angle*pi/180;
    
    
    R = getRmatrix(angles);
    %% Estimate transformation error
    [eRoppr(i,j), eRfpro(i,j), eRmbpe(i,j), eRepog(i,j) ]= estimator(m1, m2, radius, K, B, R);
    j = j + 1;

end

end

