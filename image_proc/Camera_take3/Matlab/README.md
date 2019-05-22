# Matlab Estimator - runAll

This program finds the accuracy of camera rotation estimations for simulated data according to the length of the baseline, the distance of the points to the camera, the pixel noise in the images, and also provides results with the error (in relation to the ground truth) according to the angle for a specific situations (for example, having baseline or not, having intrinsic parameters or not). It can also find the accuracy of the camera rotation estimations for real world data, possibly presenting it according to the distance of the points to the camera.

## How to run

``runAll(TYPE, vars)`` takes two arguments, the first is the type of testing desired,

- ``SIM_BASELINES`` estimate rotations with simulated points based on the length of the baseline;

- ``SIM_DISTANCES`` estimate rotations with simulated points based on distance from the camera to the points;

- ``SIM_NOISES`` estimate rotations with simulated points based on pixel noise;

- ``SIM_AXISANGLES`` estimate rotations with simulated points and observe results according to the angle executed;

- ``REAL_DISTANCES`` estimate rotation with real data based on the distance from the camera to the points;

- ``REAL_AXISANGLES`` estimate rotations with simulated points and observe results according to the angle executed;

and the other is a structure with all the necessary variables to run the program for a certain type,

- for simulation 

      sim.sigma - normal distribution sigma for generating angles
      sim.maxD - maximum distance to set the simulated points from the camera
      sim.minD - minimum distance to set the simulated points from the camera
      sim.radius - radius of the sphere to project the points in
      sim.nMatches - number of point matches to use when computing the rotation
      sim.nAngles - number of different angles to generate
      sim.intrinsics - camera's intrinsic parameters
      sim.nPixels - number of pixels to deviate as noise
      sim.saveDir - Directory where to save the results
      sim.baseline - Baseline 

    and specific variables 
    
        for ``SIM_BASELINE``

        sim.baseline.max - Maximum length of the baseline
        sim.baseline.min - Minimum length of the baseline
        sim.baseline.inc - Baseline length to increase by iteration

       for ``SIM_DISTANCES``

        sim.distance.max - Maximum distance from the points to the camera
        sim.distance.min - Minimum distance from the points to the camera
        sim.distance.inc - Distance to increase by iteration

       for ``SIM_NOISES`` 

        sim.noise.max - Maximum pixel noise to add
        sim.noise.min - Minimum pixel noise to add
        sim.noise.inc - Pixel noise to increase by iteration
      
- for real data

      real.ransac.samplePer - Percentage of points to drew the ransac model from                  
      real.ransac.enoughPer - Percentage of points enough to accept the model         
      real.ransac.maxIters - Maximum number of iterations to run ransac through                 
      real.ransac.maxErr - Maximum error to define points as inliers    
      real.baseline - Baseline
      real.radius - radius of the sphere to project the points in
      real.intrinsics - camera's intrinsic parameters
      real.minMatches - minimum number of point matches required to compute the rotation (refer to Epipolar Geometry Method)
      real.maxMatches - maximum number of point matches to use for the estimation
      real.saveDir - Directory where to save the results
      real.inputDir - Directory with the data files
          
     and specific variables 
     
      for ``REAL_DISTANCES``

      real.distance.inputDir - Directory with the data files 
      
## Implementation notes

### Real data
      
- The program reads from two data files ([previously generated](https://github.com/Mrrvm/Visual-Odometry/tree/master/image_proc/Camera_take3/input)), with the image and rotation data. 
- It uses the [SURF algorithm](https://www.mathworks.com/help/vision/ref/detectsurffeatures.html) to find the point matches between images and then [RANSAC with a Procrustes model](https://github.com/Mrrvm/Visual-Odometry/blob/master/image_proc/Camera_take3/Matlab/ransacByProcrustes.m) to decide what are the best matches to keep.

### Simulated data

### Estimating methods
#### Orthogonal Procrustes Problem (OPPR)
#### Full Procrustes (FPRO)
#### Minimization of the Back Projection Error (MBPE)
#### Epipolar Geometry

### Result plots
      
      
      
      
