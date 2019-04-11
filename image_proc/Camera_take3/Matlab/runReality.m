close all;
clear;

K = [1.1446489261369218e+03 0 9.8904229535416607e+02; 
    0 1.1452033054788860e+03 7.5547531458574417e+02;
    0 0 1];
N = 20;                      % min number of point matches
B = [0.0 0.0 0.07]';         % baseline


%% Obtain 2D points from camera images
[imgs1, imgs2] = readImages('cam_img');

for k = 1:numel(imgs1)
    %% Extract matches from images
    img1 = imgs1(k).data;
    img2 = imgs2(k).data;
    p1 = detectSURFFeatures(img1);
    p2 = detectSURFFeatures(img2);
    [f1,vpts1] = extractFeatures(img1, p1);
    [f2,vpts2] = extractFeatures(img2, p2);
    matches = matchFeatures(f1, f2, 'Method', 'Exhaustive',  'MatchThreshold', 50.0, 'Unique', true);
    m1a = vpts1(matches(:,1));
    m2a = vpts2(matches(:,2));
    figure; showMatchedFeatures(img1, img2, m1a, m2a);
    
    m1 = (K\[m1a.Location ones(m1a.Count,1)]');
    m2 = (K\[m2a.Location ones(m2a.Count,1)]');
    m1 = m1(1:2,:);
    m2 = m2(1:2,:);
    
    %% Run orthogonal procrustes problem
    [R_oppr, T_oppr] = orthProcrustesProb(m1, m2);
        
    %% Run matlab procrustes
    [R_fpro, T_fpro] = fullProcrustes(m1, m2, m1a.Count);

    %% Run minimization of back projection error
    [R_mbpe, T_mbpe] = minBackProject(double(m1), double(m2), B, double(m1a.Count), R_oppr);
        
    %% Run epipolar geometry approach
    [R_epog, T_epog] = epipolarGeo(m1, m2, K);
    
end



