close all;
clear;

K = [1.1446e+03 0                    9.8904e+02; 
        0                  1.1452e+03  7.554e+02  ;
        0                  0                    1                 ];
B              = [0.0 0.0 0.07]';         % baseline
radius = 1;

%% Obtain 2D points from camera images
[imgs1, imgs2] = readImages('../cam_img');

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
    bestInds = ransacByProcrustes(m1a.Location', m2a.Location', K, radius);
    m1Best = m1a(bestInds'>0,:);
    m2Best = m2a(bestInds'>0,:);
    figure; showMatchedFeatures(img1, img2, m1Best', m2Best');
    
    m1 = double(m1Best.Location');
    m2 = double(m2Best.Location');
    nMatches = size(m1,2);
    
    %% Run orthogonal procrustes problem
    [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);

    %% Run matlab procrustes
    [Rfpro, Tfpro] = fullProcrustes(m1, m2, radius, K);

    %% Run minimization of back projection error
    [Rmbpe, Tmbpe] = minBackProject(m1, m2, B, nMatches, Roppr, radius, K);

    %% Run epipolar geometry approach
    [Repog, Tepog] = epipolarGeo(m1, m2, radius, K);

end



