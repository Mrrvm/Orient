close all;
clear all;

vars.baseline = [0.02 -0.055 0.055]';
vars.intrinsics = [1.1446e+03 0                    9.8904e+02; 
                    0                  1.1452e+03  7.554e+02  ;
                    0                  0                    1                 ];
vars.nMatches = 20;
vars.sigma = 10;
vars.minD = 0.05;
vars.imgDim = [2056 1542];
vars.nAngles = 100;
vars.nPixels = 0;

vars.maxD = 0.24;
vars.radius = vars.maxD + 1;
vars.saveDir = 'workshop/results/sim/d0.24/';
%runAll('SIM_AXISANGLES', vars);

vars.maxD = 5.77;
vars.radius = vars.maxD + 1;
vars.saveDir = 'workshop/results/sim/d5/';
%runAll('SIM_AXISANGLES', vars);

vars.tanDist = [2.8362364785395738e-05 -4.0722129540613915e-04];
vars.radialDist = [-3.1095067022606104e-01 1.5512159682150103e-01 -5.0836375118168069e-02];
vars.ransac.samplePer = 0.4;                  
vars.ransac.enoughPer = 0.6;              
vars.ransac.maxIters    = 20;                 
vars.ransac.maxErr       = 0.01;       
vars.minMatches = 20;
vars.maxMatches = 20;
vars.radius = 0.24 + 1;
vars.inputDir = 'workshop/data/d0.24/';
vars.saveDir = 'workshop/results/real/d0.24/';
runAll('REAL_AXISANGLES', vars);

vars.radius = 5 + 1;
vars.inputDir = 'workshop/data/d5/';
vars.saveDir = 'workshop/results/real/d5/';
%runAll('REAL_AXISANGLES', vars);




