#include "defs.h"

bool IsRotm(Mat rotm) {
    Mat Rt;
    transpose(rotm, Rt);
    Mat shouldBeIdentity = Rt * rotm;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return norm(I, shouldBeIdentity) < 1e-6;
}

Mat Eul2Rotm(Mat eul) {
    Mat Rx = (Mat_<double>(3,3) <<
            1,       0,              0,
            0,       cos(eul.at<double>(0)),  -sin(eul.at<double>(0)),
            0,       sin(eul.at<double>(0)),   cos(eul.at<double>(0))
    );
    Mat Ry = (Mat_<double>(3,3) <<
             cos(eul.at<double>(1)),  0,      sin(eul.at<double>(1)),
             0,                          1,      0,
            -sin(eul.at<double>(1)),  0,      cos(eul.at<double>(1))
    );
    Mat Rz = (Mat_<double>(3,3) <<
            cos(eul.at<double>(2)),    -sin(eul.at<double>(2)),   0,
            sin(eul.at<double>(2)),     cos(eul.at<double>(2)),   0,
            0,                              0,                         1);
    Mat rotm = Rz * Ry * Rx;
    return rotm;
}

Mat Rotm2Eul(Mat rotm) {

    Mat eul;

    float sy = sqrt(rotm.at<double>(0,0)*rotm.at<double>(0,0) +  rotm.at<double>(1,0)*rotm.at<double>(1,0));
    bool singular = sy < 1e-6; // If
    double x, y, z;
    if (!singular) {
        x = atan2( rotm.at<double>(2,1), rotm.at<double>(2,2));
        y = atan2(-rotm.at<double>(2,0), sy);
        z = atan2( rotm.at<double>(1,0), rotm.at<double>(0,0));
    }
    else {
        x = atan2(-rotm.at<double>(1,2), rotm.at<double>(1,1));
        y = atan2(-rotm.at<double>(2,0), sy);
        z = 0;
    }
    eul = (Mat_<double>(1,3) << x, y, z);
    return eul;
}

// TODO: implement rest of functions according to efficiency of conversion
