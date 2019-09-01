#include "rotation.h"

Rotation::Rotation() {}


Rotation::Rotation(Mat _baseline, Mat _intrinsics) {
    baseline = _baseline;
    intrinsics = _intrinsics;
}

bool Rotation::Estimate(Mat m1, Mat m2, string method) {

    if (method == "PROC") {

    }
    else if(method == "GRAT") {

    }
    else if(method == "MBPE") {

    }

    return true;
}

bool ProjectOnSphere(Mat m1, Mat m2, Mat mp1, Mat mp2) {

}
