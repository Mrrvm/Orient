close all;
clear all;

vars.baseline = [0.02 -0.055 0.055]';
vars.intrinsics = [1.1573e+03 -3.3579         975.9459; 
                           0                   1.1584e+03  798.4888;
                           0                   0                    1                 ];
vars.tanDist = [3.0406e-04 7.1815e-04];
vars.radialDist = [-0.3160 0.1699 -0.0569];
vars.nMatches = 20;
vars.sigma = 10;
vars.minD = 0.05;
vars.imgDim = [2056 1542];
vars.nAngles = 100;
vars.nPixels = 5;

vars.maxD = 0.24;
vars.radius = vars.maxD + 1;
vars.saveDir = '../results/Matlab/sim/d0.24/';
runAll('SIM_AXISANGLES', vars);

vars.maxD = 5.77;
vars.radius = vars.maxD + 1;
vars.saveDir = '../results/Matlab/sim/d5/';
%runAll('SIM_AXISANGLES', vars);

vars.ransac.samplePer = 0.4;                  
vars.ransac.enoughPer = 0.6;              
vars.ransac.maxIters    = 20;                 
vars.ransac.maxErr       = 0.005;       
vars.minMatches = 20;
vars.maxMatches = 20;
vars.radius = 0.24 + 1;
vars.inputDir = 'workshop/data/d0.24/';
vars.saveDir = 'results/real/d0.24/';
%runAll('REAL_AXISANGLES', vars);

vars.radius = 5 + 1;
vars.inputDir = 'workshop/data/d5/';
vars.saveDir = 'results/real/d5/';
%runAll('REAL_AXISANGLES', vars);




