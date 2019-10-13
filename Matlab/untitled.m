intrinsics = [1.1253e+03 0.945541684501329     9.960929796822086e+02;
              0		   1.125630873153923e+03 7.543243830849593e+02;
              0          0                     1                   ];
tanDist = [4.7003e-04 -3.3083e-04];
radialDist = [-0.3007 0.1288 -0.0318];
squareSize = 20;
vars.minMatches = 3;
vars.maxMatches = 20;
vars.ransac.on = 0;
vars.ransac.outlierPer          = 0.70; 
vars.ransac.goodMatches    = round(vars.maxMatches*0.5);
vars.ransac.maxErr              = 0.001;   % in meters
radius = 3;

cameraParams = cameraParameters('IntrinsicMatrix', intrinsics', 'RadialDistortion', radialDist, 'TangentialDistortion', tanDist); 

img1 = undistortImage(imread('/home/imarcher/image1.png'), cameraParams);
img2 = undistortImage(imread('/home/imarcher/image2.png'), cameraParams);
p1 = detectSURFFeatures(img1);
p2 = detectSURFFeatures(img2);
[f1,vpts1] = extractFeatures(img1, p1);
[f2,vpts2] = extractFeatures(img2, p2);
matches = matchFeatures(f1, f2, 'Method', 'Exhaustive',  'MatchThreshold', 100, 'Unique', true);
m1a = vpts1(matches(:,1));
m2a = vpts2(matches(:,2));
[m1, m2, err, ~] = ransacByProcrustes(m1a.Location', m2a.Location', intrinsics, radius, vars.minMatches, vars.maxMatches, vars.ransac);
    
figure; showMatchedFeatures(img1, img2, m1', m2');  

[R, T] = orthProcrustesProb(m1, m2, radius, intrinsics);
rotm2eul(R)*180/pi