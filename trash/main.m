close all;
clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SIMULATION

I = [1 0 0; 0 1 0; 0 0 1];
baseline = [0.02 -0.055 0.055]';
focalLength = 1;
m2pix = [3779.53 3779.53];     
skew = 0;
axisOffset = [0 0];  
sim.sigma = 5;
sim.maxD = 5;
sim.minD = 2.5;
sim.radius = 1;
sim.nMatches = 20;
sim.imgDim = [2056 1542];

% Test baseline effect
sim.nAngles = 100;
sim.intrinsics = I;
sim.noisePixels = 0;
% Angle per Axis, baseline OFF, K=I
sim.baseline = [0.0 0.0 0.0]';  
sim.saveDir = 'results/simulation/angleaxis_baselineOFF_K==I/';
runAll('SIM_AXISANGLES', sim);
% Angle per Axis, baseline ON, K=I
sim.baseline = baseline;
sim.saveDir = 'results/simulation/angleaxis_baselineON_K==I/';
runAll('SIM_AXISANGLES', sim);

%% Test baseline effect with intrinsics
sim.nAngles = 20;
sim.intrinsics =  [focalLength*m2pix(1)   skew                               axisOffset(1); 
                           0                                   focalLength*m2pix(2)     axisOffset(2); 
                           0                                   0                                     1                 ];
sim.noisePixels = 0;
% Angle per Axis, baseline OFF with pixel noise ON (K!=I)
sim.baseline = [0.0 0.0 0.0]';  
sim.saveDir = 'results/simulation/angleaxis_baselineOFF_K!=I/';
runAll('SIM_AXISANGLES', sim);
% Angle per Axis, baseline ON with pixel noise ON (K!=I)
sim.baseline = baseline;  
sim.saveDir = 'results/simulation/angleaxis_baselineON_K!=I/';
runAll('SIM_AXISANGLES', sim);

%% Test distance of points to camera effect w/o baseline
sim.nAngles = 20;
sim.intrinsics = I;
sim.noisePixels = 0;
sim.distance.max = 20;
sim.distance.min = 0.05;
sim.distance.inc = 0.5;
% Distances with baseline OFF 
sim.baseline = [0.0 0.0 0.0]';  
sim.saveDir = 'results/simulation/distance_baselineOFF_K==I/';
runAll('SIM_DISTANCES', sim);
% Distances with baseline ON
sim.baseline = baseline;  
sim.saveDir = 'results/simulation/distance_baselineON_K==I/';
runAll('SIM_DISTANCES', sim);

%% Test pixel noise effect w/o baseline
sim.nAngles = 50;
sim.intrinsics =  [focalLength*m2pix(1)   skew                               axisOffset(1); 
                           0                                   focalLength*m2pix(2)     axisOffset(2); 
                           0                                   0                                     1                 ];
sim.noise.max = 6;
sim.noise.min = 1;
sim.noise.inc = 1;                
% Noise with baseline OFF 
sim.baseline = [0.0 0.0 0.0]';  
sim.saveDir = 'results/simulation/noise_baselineOFF_K!=I/';
runAll('SIM_NOISES', sim);
% Noise with baseline ON
sim.baseline = baseline;  
sim.saveDir = 'results/simulation/noise_baselineON_K!=I/';
runAll('SIM_NOISES', sim);

%% Test baseline length effect 
sim.nAngles = 50;
sim.intrinsics = I;
sim.noisePixels = 0;
sim.baseline.max = [0.0 0.0 1]';
sim.baseline.min = [0.0 0.0 0.0]';
sim.baseline.inc = [0.0 0.0 0.1]';
sim.saveDir = 'results/simulation/baselines_K==I/';
runAll('SIM_BASELINES', sim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%REAL DATA

%% Test angle per axis with real data from Motive 
% at 0.24m
real.ransac.samplePer = 0.2;                  
real.ransac.enoughPer = 0.9;              
real.ransac.maxIters    = 20;                 
real.ransac.maxErr       = 0.005;              
real.baseline                = [0.02 -0.055 0.055]';
real.radius        = 1;
real.intrinsics = [1.1446e+03 0                    9.8904e+02; 
                            0                  1.1452e+03  7.554e+02  ;
                            0                  0                    1                 ];
real.minMatches = 20;
real.maxMatches = 50;
real.saveDir = 'results/realdata/d0.24/';
real.inputDir = '../input/Motive/filteredata/d0.24/';
runAll('REAL_AXISANGLES', real);
% at ~5m
real.saveDir = 'results/realdata/d5/';
real.inputDir = '../input/Motive/filteredata/d5/';
runAll('REAL_AXISANGLES', real);

%% Test distances with real data from motive
real.distance.inputDir = '../input/Motive/filteredata/'; 
real.saveDir = 'results/realdata/distances/';
runAll('REAL_DISTANCES', real);


