close all;
clear all;

vars.methods = {'OPPR', 'MEEN', 'MBPE', 'GRAT'};
vars.colors = {'b', 'y', 'r', 'c'};
vars.filename = 'results.txt';
vars.baseline = [0.02 -0.055 0.055]';
vars.intrinsics = [1.1573e+03 -3.3579         975.9459; 
                           0                   1.1584e+03  798.4888;
                           0                   0                    1                 ];
vars.tanDist = [3.0406e-04 7.1815e-04];
vars.radialDist = [-0.3160 0.1699 -0.0569];
vars.imgDim = [2056 1542];
vars.minMatches = 3;
vars.maxMatches = 20;
vars.ransac.on = 1;
vars.ransac.outlierPer          = 0.40; 
vars.ransac.goodMatches    = round(vars.maxMatches*0.5);
vars.ransac.maxErr              = 0.005;   % in meters

%==========================================================================
% SIMULATION
vars.nMatches = 50;
vars.nFalseMatches = 3;
vars.saccadeDistrSigma = 10;
vars.minDistToCam = 0.05;
vars.nSaccades = 10;
vars.nNoisePixels = 10;

vars.maxDistToCam= 0.24;
vars.projectionRadius = vars.maxDistToCam + 1;
vars.saveDir = '../results/Matlab/sim/d0.24/';
runAll('SIM_AXISANGLES', vars);

vars.maxDistToCam = 5.77;
vars.projectionRadius = vars.maxDistToCam + 1;        
vars.saveDir = '../results/Matlab/sim/d5/';
%runAll('SIM_AXISANGLES', vars);

vars.noise.max = 700;
vars.noise.min = 600;
vars.noise.inc = 30; 
vars.saveDir = '../results/Matlab/sim/d0.24/noise/';
%runAll('SIM_NOISES', vars);

%==========================================================================
% REAL DATA 
vars.maxDistToCam= 0.24;
vars.projectionRadius = vars.maxDistToCam + 1;
vars.inputDir = 'workshop/data/d0.24/';
vars.saveDir = 'results/real/d0.24/';
%runAll('REAL_AXISANGLES', vars);

vars.maxDistToCam = 5.77;
vars.projectionRadius = vars.maxDistToCam + 1;        
vars.inputDir = 'workshop/data/d5/';
vars.saveDir = 'results/real/d5/';
%runAll('REAL_AXISANGLES', vars);




