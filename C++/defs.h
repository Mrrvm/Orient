#ifndef MAIN_CPP_DEFS_H
#define MAIN_CPP_DEFS_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/aruco/charuco.hpp"

#define HEIGHT 1542
#define WIDTH 2056
#define BITSPIXEL 32
#define CVTYPE CV_8UC4
#define MAXMATCHES 20
#define MINMATCHES 3
#define MINHESSIAN 800
#define MAXERROR 0.005
#define OUTLIERPER 0.7

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

using namespace std;
using namespace cv;

bool IsRotm(Mat);
Mat Eul2Rotm(Mat);
Mat Rotm2Eul(Mat);
Mat Rotm2Quat(Mat);
Mat Quat2Rotm(Mat);
Mat Angax2Rotm(Mat);
Mat Rotm2Angax(Mat);
Mat Quat2Eul(Mat);
Mat QuatConjugate(Mat);
Mat QuatMultiply(Mat, Mat);

#endif
