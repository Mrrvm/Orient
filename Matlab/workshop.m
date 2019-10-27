close all;
clear all;

vars.methods = {'OPPR', 'MBPE', 'GRAT'};
vars.colors = {'b', 'g', 'r'};
vars.filename = 'results.txt';
vars.currBaseline = [0.0 0.0 0.0537]'; %eye
%vars.currBaseline = [0.015 0.051 0.02275]'; %woodstand 
%vars.currBaseline = [0.0 0.0 0.0]';
vars.intrinsics = [1.1253e+03 0.945541684501329     9.960929796822086e+02;
                     0		  1.125630873153923e+03 7.543243830849593e+02;
                     0        0                     1                    ];
vars.tanDist = [4.7003e-04 -3.3083e-04];
vars.radialDist = [-0.3007 0.1288 -0.0318];
vars.imgDim = [2056 1542];
vars.minMatches = 3;
vars.maxMatches = 30;
vars.ransac.on = 1;
vars.ransac.outlierPer          = 0.40; 
vars.ransac.goodMatches    = round(vars.maxMatches*0.5);
vars.ransac.maxErr              = 0.05;   % in meters
%==========================================================================
% SIMULATION
vars.nMatches = 30;
vars.nFalseMatches = round(vars.nMatches*0.1);
vars.saccadeSigma = 10;
vars.nSaccades = 45;
vars.currDistToCam.min = 0.05;
vars.currNoisePixelsSigma = 10;
vars.currDistToCam.max = 5;
vars.projectionRadius = vars.currDistToCam.max + 1;  

vars.distToCam.max = 10;
vars.distToCam.min = 0.05;
vars.distToCam.inc = 1;
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Depth/';
runAll('SIM_DISTANCES', vars);

vars.noisePixelsSigma.max = 100;
vars.noisePixelsSigma.min = 0;
vars.noisePixelsSigma.inc = 10; 
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Noise/';
%runAll('SIM_NOISES', vars);
      
vars.saveDir = '../results/Matlab/sim/Ex-Eye/';
%runAll('SIM_AXISANGLES', vars);


%==========================================================================
% REAL DATA 
vars.currDistToCam.max = 5;
vars.projectionRadius = vars.currDistToCam.max + 1;        
vars.inputDir = '../input/camera/Ex1-Lab-woodstand/y/';
vars.saveDir = '../results/Matlab/real/camera/Ex1-Lab-woodstand/y/';
%runAll('REAL_AXISANGLES', vars);

