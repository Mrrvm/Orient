intrinsics = [1.1253e+03 0.945541684501329     9.960929796822086e+02;
              0		   1.125630873153923e+03 7.543243830849593e+02;
              0          0                     1                   ];
tanDist = [4.7003e-04 -3.3083e-04];
radialDist = [-0.3007 0.1288 -0.0318];
squareSize = 21;
cameraParams = cameraParameters('IntrinsicMatrix', intrinsics, 'RadialDistortion', radialDist, 'TangentialDistortion', tanDist); 

img1 = imread('/home/imarcher/0d.png');
img2 = imread('/home/imarcher/y15.39-.png');

[imgPts1, boardSize] = detectCheckerboardPoints(img1);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[R1, t1] = extrinsics(imgPts1, worldPoints, cameraParams);
[imgPts2, boardSize] = detectCheckerboardPoints(img2);
[R2, t2] = extrinsics(imgPts2, worldPoints, cameraParams);
R = R1*R2';
rotm2eul(R)*180/pi
