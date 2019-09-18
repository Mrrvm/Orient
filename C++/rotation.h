#ifndef MAIN_CPP_ROTATION_H
#define MAIN_CPP_ROTATION_H

#include "defs.h"
#include "image.h"
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/core/eigen.hpp"

using namespace ceres;
using namespace Eigen;

class Rotation {

    Mat MakeHomogeneous(Mat);
    bool GRAT(Mat, Mat, Mat, bool);
    bool MBPE(Mat, Mat, Mat, Mat depthinit, bool);

public:
    Mat baseline;
    Mat intrinsics;
    int radius;
    Mat rotm;
    Mat eul;
    Mat quat;

    Rotation(Mat, Mat, int);
    Mat ProjectToSphere(Mat);
    Mat ProjectToPlane(Mat);
    bool Procrustes(Mat, Mat);
    bool Estimate(Mat, Mat, string, Mat eulinit = Mat(1,1, DataType<double>::type), bool info = false);
    double ComputeError(Mat);

};

#endif
