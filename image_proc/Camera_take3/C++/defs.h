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
#include <ueye.h>

#define IMGSPATH "/home/imarcher/Dropbox/Tecnico/Visual-Odometry/image_proc/Camera_take3/cam_img/"


#define COLOR false
#define HEIGHT 1542
#define WIDTH 2056

#if COLOR
#define BITSPIXEL 32
#define CVTYPE CV_8UC4
#endif
#if !(COLOR)
#define BITSPIXEL 8
#define CVTYPE CV_8UC1
#endif

using namespace std;
using namespace cv;

void SpawnError(string where);

#endif
