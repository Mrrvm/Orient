#ifndef MAIN_CPP_ROTATION_H
#define MAIN_CPP_ROTATION_H

#include "defs.h"
#include "image.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "opencv2/core/eigen.hpp"

using namespace Eigen;
using namespace ceres;

class Rotation {

    bool GRAT();
    bool MBPE();
    MatrixXd ProjectToSphere(MatrixXd);

public:
    Image img1;
    Image img2;
    Vector3d baseline;
    Matrix3d intrinsics;
    int radius;
    Matrix3d rotm;
    Vector3d eul;
    Quaterniond quat;

    Rotation();
    Rotation(Mat, Mat, int);
    Mat ProjectToSphere(Mat);
    bool Procrustes(Mat, Mat);
    bool Procrustes(MatrixXd, MatrixXd);
    bool Estimate(Mat, Mat, string);
    bool Estimate(MatrixXd, MatrixXd, string);
};


#endif
