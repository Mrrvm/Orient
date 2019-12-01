close all;
clear all;

vars.methods = {'OPPR', 'MBPE', 'GRAT'};
vars.colors = {'b', 'r', 'g'};
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
vars.saccadeSigma = 4;
vars.nSaccades = 45;
vars.currDistToCam.min = 0.05;
vars.currNoisePixelsSigma = 10;
vars.currDistToCam.max = 5;
vars.projectionRadius = vars.currDistToCam.max + 1;  

vars.axis = 'all';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-10deg/';
%runAll('SIM_AXISANGLES', vars);
vars.axis = 'x';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-10deg/x/';
%runAll('SIM_AXISANGLES', vars);
vars.axis = 'y';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-10deg/y/';
%runAll('SIM_AXISANGLES', vars);
vars.axis = 'z';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-10deg/z/';
%runAll('SIM_AXISANGLES', vars);

vars.saccadeSigma = 15;
vars.axis = 'all';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-45deg/';
runAll('SIM_AXISANGLES', vars);
vars.axis = 'x';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-45deg/x/';
%runAll('SIM_AXISANGLES', vars);
vars.axis = 'y';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-45deg/y/';
%runAll('SIM_AXISANGLES', vars);
vars.axis = 'z';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-45deg/z/';
%runAll('SIM_AXISANGLES', vars);

vars.saccadeSigma = 4;
vars.distToCam.max = 10;
vars.distToCam.min = 0.05;
vars.distToCam.inc = 1;
vars.axis = 'all';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Depth/';
%runAll('SIM_DISTANCES', vars);
vars.axis = 'x';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Depth/x/';
%runAll('SIM_DISTANCES', vars);
vars.axis = 'y';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Depth/y/';
%runAll('SIM_DISTANCES', vars);
vars.axis = 'z';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Depth/z/';
%runAll('SIM_DISTANCES', vars);

vars.saccadeSigma = 4;
vars.baseline.max = 0.5;
vars.baseline.min = 0.05;
vars.baseline.inc = 0.1; 
vars.axis = 'all';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Baseline/';
%runAll('SIM_BASELINES', vars);

vars.axis = 'all';
vars.saccadeSigma = 4;
vars.ransac.on = 1;
vars.ransac.goodMatches = 3;
vars.noisePixelsSigma.max = 110;
vars.noisePixelsSigma.min = 0;
vars.noisePixelsSigma.inc = 10; 
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Noise/';
%runAll('SIM_NOISES', vars);

vars.nFalseMatches = 0;
vars.currNoisePixelsSigma = 0;
vars.ransac.on = 0;
vars.saccadeSigma = 15;
vars.axis = 'all';
vars.saveDir = '../results/Matlab/sim/Ex-Eye-Ransac/without/';
%runAll('SIM_AXISANGLES', vars);

%==========================================================================
% REAL DATA 
vars.currDistToCam.max = 5;
vars.projectionRadius = vars.currDistToCam.max + 1;        
vars.inputDir = '../input/camera/Ex2-Lab-Eye/';
vars.saveDir = '../results/Matlab/real/camera/Ex2-Lab-Eye/';
%runAll('REAL_AXISANGLES', vars);



