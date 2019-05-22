# Matlab Estimator - runAll

This program finds the accuracy of camera rotation estimations for simulated data according to the length of the baseline, the distance of the points to the camera, the pixel noise in the images, and also provides results with the error (in relation to the ground truth) according to the angle for a specific situations (for example, having baseline or not, having intrinsic parameters or not). It can also find the accuracy of the camera rotation estimations for real world data, possibly presenting it according to the distance of the points to the camera.

## Objective

The main goal is two evaluate the 4 different estimation methods, this is just a time saver.

- [How to run](#how-to-run)
- [Implementation notes](#implementation-notes)
  * [Using real data](#using-real-data)
  * [Using simulated data](#using-simulated-data)
  * [Estimation methods](#estimation-methods)
    + [Orthogonal Procrustes Problem (OPPR)](#orthogonal-procrustes-problem)
    + [Full Procrustes (FPRO)](#full-procrustes)
    + [Minimization of the Back Projection Error (MBPE)](#minimization-of-the-back-projection-error)
    + [Epipolar Geometry](#epipolar-geometry)
  * [Comparing Results](#comparing-results)
  * [Time concerns](#time-concerns)
- [How to define the parameters](#how-to-define-the-parameters)
- [My obtained results](#my-obtained-results)


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
      real.minMatches - minimum number of point matches required to compute the rotation
      real.maxMatches - maximum number of point matches to use for the estimation
      real.saveDir - Directory where to save the results
      real.inputDir - Directory with the data files
          
     and specific variables 
     
      for ``REAL_DISTANCES``

      real.distance.inputDir - Directory with the data files 
      
## Implementation notes

### Using real data
      
- The program reads from two data files ([previously generated](https://github.com/Mrrvm/Visual-Odometry/tree/master/input)), with the image and rotation data. 
- It uses the [SURF algorithm](https://www.mathworks.com/help/vision/ref/detectsurffeatures.html) to find the point matches between images and then [RANSAC with a Procrustes model](https://github.com/Mrrvm/Visual-Odometry/blob/master/Matlab/ransacByProcrustes.m) to decide what are the best matches to keep.

### Using simulated data
In order to produce simulated points, a few steps are taken,

1) Generate a number, ``sim.nAngles``, of different angles in a normal distribution with sigma, ``sim.sigma``.

Now per each axis, per each of the generated angles, 

2) Generate a number, ``nMatches``, of different 3D points within a 90º degree viewfield seen from the camera. This will be the first pointcloud, M1.

3)  Rotate the point cloud by the generated angle. The result will be the second pointcloud, M2.

4) Convert this points to image points, m1 and m2.

![](https://github.com/Mrrvm/Visual-Odometry/blob/master/images/simulation_diagram.png)

Finally, this points may be used as if it were images to 

5) Estimate the transformation and get results by comparing with the ground truth.

6) Return to  2)

### Estimation methods
#### Orthogonal Procrustes Problem 
Minimizes the difference between the pointclouds M1 and M2 through

<a href="https://www.codecogs.com/eqnedit.php?latex=\left&space;\|&space;M_1&space;-&space;RM_2&space;\right&space;\|^2" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\left&space;\|&space;M_1&space;-&space;RM_2&space;\right&space;\|^2" title="\left \| M_1 - RM_2 \right \|^2" /></a>

which may be expanded as 

<a href="https://www.codecogs.com/eqnedit.php?latex=\left&space;\|&space;M_1&space;-&space;RM_2&space;\right&space;\|^2&space;=&space;trace(M_1^T&space;M_1&space;&plus;&space;M_2^T&space;M_2&space;)&space;−&space;2&space;trace(M_2^T&space;M_1&space;R)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\left&space;\|&space;M_1&space;-&space;RM_2&space;\right&space;\|^2&space;=&space;trace(M_1^T&space;M_1&space;&plus;&space;M_2^T&space;M_2&space;)&space;−&space;2&space;trace(M_2^T&space;M_1&space;R)" title="\left \| M_1 - RM_2 \right \|^2 = trace(M_1^T M_1 + M_2^T M_2 ) − 2 trace(M_2^T M_1 R)" /></a>.

So minimizing is the same as maximizing the second term. Applying the SVD on the second term, 

<a href="https://www.codecogs.com/eqnedit.php?latex=trace(M_2^T&space;M_1&space;R)&space;=&space;trace(U&space;\Sigma&space;V^T&space;R)&space;=&space;trace(\Sigma&space;H)&space;=&space;\sum^N_{i=1}&space;\sigma_i&space;h_{ii}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?trace(M_2^T&space;M_1&space;R)&space;=&space;trace(U&space;\Sigma&space;V^T&space;R)&space;=&space;trace(\Sigma&space;H)&space;=&space;\sum^N_{i=1}&space;\sigma_i&space;h_{ii}" title="trace(M_2^T M_1 R) = trace(U \Sigma V^T R) = trace(\Sigma H) = \sum^N_{i=1} \sigma_i h_{ii}" /></a>.

Because the values <a href="https://www.codecogs.com/eqnedit.php?latex=\sigma_i" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sigma_i" title="\sigma_i" /></a> are all non negative, the expression is its maximum when <a href="https://www.codecogs.com/eqnedit.php?latex=h_{ii}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?h_{ii}" title="h_{ii}" /></a> is at 1, so R may be obtained through <a href="https://www.codecogs.com/eqnedit.php?latex=I&space;=&space;V^TRU" target="_blank"><img src="https://latex.codecogs.com/gif.latex?I&space;=&space;V^TRU" title="I = V^TRU" /></a>.

Hence, the implentation is a simple SVD of <a href="https://www.codecogs.com/eqnedit.php?latex=M_2^TM_1" target="_blank"><img src="https://latex.codecogs.com/gif.latex?M_2^TM_1" title="R = VU^T" /></a> and obtaining of <a href="https://www.codecogs.com/eqnedit.php?latex=R&space;=&space;VU^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?R&space;=&space;VU^T" title="R = VU^T" /></a>.

#### Full Procrustes
#### Minimization of the Back Projection Error
#### Epipolar Geometry

### Comparing Results

### Time concerns
      
## How to define the parameters
      
## My obtained results 
      
