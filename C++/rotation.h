#ifndef MAIN_CPP_ROTATION_H
#define MAIN_CPP_ROTATION_H

#include "defs.h"
#include "image.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace ceres;

class Rotation {

    Mat baseline;
    Mat intrinsics;

    bool Procrustes();
    bool GRAT();
    bool MBPE();

public:
    Image img1;
    Image img2;
    Mat rotm;
    Mat eul;
    Mat quat;

    Rotation();
    Rotation(Mat, Mat);
    bool Estimate(Mat, Mat, string);
    bool ProjectOnSphere(Mat, Mat, Mat, Mat);
};


#endif
