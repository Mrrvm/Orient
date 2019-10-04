close all;
clear all;

vars.methods = {'OPPR', 'GRAT', 'MBPE'};
vars.peraxis = 0;
vars.colors = {'b', 'g', 'r'};
vars.filename = 'results.txt';
vars.currBaseline = [0.0 0.0 0.08479]'; % add the 4mm lens length
vars.intrinsics = [1.1253e+03 0.945541684501329     9.960929796822086e+02;
                     0		  1.125630873153923e+03 7.543243830849593e+02;
                     0        0                     1                    ];
vars.tanDist = [4.7003e-04 -3.3083e-04];
vars.radialDist = [-0.3007 0.1288 -0.0318];
vars.imgDim = [2056 1542];
vars.minMatches = 3;
vars.maxMatches = 20;
vars.ransac.on = 1;
vars.ransac.outlierPer = 0.40; 
vars.ransac.goodMatches = round(vars.maxMatches*0.5);
vars.ransac.maxErr = 0.05;   % in meters

vars.currDistToCam.max = 3;
vars.projectionRadius  = vars.currDistToCam.max + 1;
vars.inputDir = '../input/Chessboard/filterdata/Ex1-Lab/';
vars.saveDir = '/home/imarcher/Ex1-Lab_results/';
runAll('REAL_AXISANGLES', vars);

