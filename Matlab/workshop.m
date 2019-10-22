close all;
clear all;

vars.methods = {'OPPR', 'MBPE', 'GRAT'};
vars.colors = {'b', 'g', 'r'};
vars.filename = 'results.txt';
vars.currBaseline = [0.0 0.0 0.0537]';
vars.intrinsics = [1.1253e+03 0.945541684501329     9.960929796822086e+02;
                     0		  1.125630873153923e+03 7.543243830849593e+02;
                     0        0                     1                    ];
vars.tanDist = [4.7003e-04 -3.3083e-04];
vars.radialDist = [-0.3007 0.1288 -0.0318];
vars.imgDim = [2056 1542];
vars.minMatches = 3;
vars.maxMatches = 30;
vars.ransac.on = 1;
vars.ransac.outlierPer          = 0.70; 
vars.ransac.goodMatches    = round(vars.maxMatches*0.5);
vars.ransac.maxErr              = 0.001;   % in meters

%==========================================================================
% REAL DATA 

vars.entropyThreshold = 1;
vars.peraxis = 0;
vars.currDistToCam.max = 5;
vars.projectionRadius = vars.currDistToCam.max + 1;        
vars.inputDir = '../input/camera/Ex2-Lab-Eye/';
vars.saveDir = '../results/Matlab/real/camera/Ex2-Lab-Eye/';
runAll('REAL_AXISANGLES', vars);

%==========================================================================
% SIMULATION
vars.nMatches = 20;
vars.nFalseMatches = round(vars.nMatches*0.1);
vars.saccadeSigma = 10;
vars.nSaccades = 45;
vars.currDistToCam.min = 0.05;
vars.currNoisePixelsSigma = 10;

vars.currDistToCam.max = 5;
vars.projectionRadius = vars.currDistToCam.max + 1;        
vars.saveDir = '../results/Matlab/sim/Ex1-Lab-woodstand/';
%runAll('SIM_AXISANGLES', vars);

vars.currDistToCam.max= 1;
vars.projectionRadius = vars.currDistToCam.max + 1;  
vars.noisePixelsSigma.max = 200;
vars.noisePixelsSigma.min = 0;
vars.noisePixelsSigma.inc = 10; 
vars.saveDir = '../results/Matlab/sim/d1/noise/';
%runAll('SIM_NOISES', vars);



