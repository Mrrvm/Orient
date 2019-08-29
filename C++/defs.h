#ifndef MAIN_CPP_DEFS_H
#define MAIN_CPP_DEFS_H

#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"

#define HEIGHT 1542
#define WIDTH 2056
#define BITSPIXEL 32
#define CVTYPE CV_8UC4

#define MINHESSIAN 300

using namespace std;
using namespace cv;

void SpawnError(string where);
double Eul2Rotm();
vector<double> Rotm2Eul();


#endif
