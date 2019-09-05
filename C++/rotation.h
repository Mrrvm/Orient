#ifndef MAIN_CPP_ROTATION_H
#define MAIN_CPP_ROTATION_H

#include "defs.h"
#include "image.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/core/eigen.hpp"

using namespace ceres;

class Rotation {

    bool GRAT(Mat, Mat, Mat);
    bool MBPE(Mat, Mat, Mat);

public:
    Image img1;
    Image img2;
    double* baseline;
    Mat intrinsics;
    int radius;
    Mat rotm;
    Mat eul;
    Mat quat;

    Rotation(double*, Mat, int);
    Mat ProjectToSphere(Mat);
    bool Procrustes(Mat, Mat);
    bool Estimate(Mat, Mat, Mat, string);

};


#endif
