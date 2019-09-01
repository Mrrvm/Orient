#include "rotation.h"

Rotation::Rotation() {}


Rotation::Rotation(Mat _baseline, Mat _intrinsics) {
    baseline = _baseline;
    intrinsics = _intrinsics;
}

bool Rotation::Estimate(Mat m1, Mat m2, string method) {

}

