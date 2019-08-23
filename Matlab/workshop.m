close all;
clear all;

vars.methods = {'OPPR', 'MEEN', 'MBPE', 'GRAT', 'MEE'};
vars.colors = {'b', 'g', 'r', 'c', 'm'};
vars.filename = 'results.txt';
vars.currBaseline = [0.02 -0.055 0.055]';
vars.intrinsics = [1.1573e+03 -3.3579         975.9459; 
                           0                   1.1584e+03  798.4888;
                           0                   0                    1                 ];
vars.tanDist = [3.0406e-04 7.1815e-04];
vars.radialDist = [-0.3160 0.1699 -0.0569];
vars.imgDim = [2056 1542];
vars.minMatches = 3;
vars.maxMatches = 20;
vars.ransac.on = 0;
vars.ransac.outlierPer          = 0.40; 
vars.ransac.goodMatches    = round(vars.maxMatches*0.5);
vars.ransac.maxErr              = 0.05;   % in meters

%==========================================================================
% SIMULATION
vars.nMatches = 20;
vars.nFalseMatches = 1;
vars.saccadeSigma = 10;
vars.nSaccades = 10;
vars.currDistToCam.min = 0.05;
vars.currNoisePixelsSigma = 10;

vars.currDistToCam.max= 0.24;
vars.projectionRadius = vars.currDistToCam.max + 1;
vars.saveDir = '../results/Matlab/sim/d0.24/';
%runAll('SIM_AXISANGLES', vars);

vars.currDistToCam.max = 5.77;
vars.projectionRadius = vars.currDistToCam.max + 1;        
vars.saveDir = '../results/Matlab/sim/d5/';
%runAll('SIM_AXISANGLES', vars);

vars.currDistToCam.max= 1;
vars.projectionRadius = vars.currDistToCam.max + 1;  
vars.noisePixelsSigma.max = 200;
vars.noisePixelsSigma.min = 0;
vars.noisePixelsSigma.inc = 10; 
vars.saveDir = '../results/Matlab/sim/d1/noise/';
runAll('SIM_NOISES', vars);

%==========================================================================
% REAL DATA 
vars.currDistToCam.max= 0.24;
vars.projectionRadius = vars.currDistToCam.max + 1;
vars.inputDir = 'workshop/data/d0.24/';
vars.saveDir = 'results/real/d0.24/';
%runAll('REAL_AXISANGLES', vars);

vars.currDistToCam.max = 5.77;
vars.projectionRadius = vars.currDistToCam.max + 1;        
vars.inputDir = 'workshop/data/d5/';
vars.saveDir = 'results/real/d5/';
%runAll('REAL_AXISANGLES', vars);




