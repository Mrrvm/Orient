#include "defs.h"

bool IsRotm(Mat rotm) {
    Mat Rt;
    transpose(rotm, Rt);
    Mat shouldBeIdentity = Rt * rotm;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return norm(I, shouldBeIdentity) < 1e-6;
}

Mat Eul2Rotm(Mat eul) {

    double eulx = eul.at<double>(2);
    double euly = eul.at<double>(1);
    double eulz = eul.at<double>(0);

    double cx = cos(eulx), cy = cos(euly), cz = cos(eulz);
    double sx = sin(eulx), sy = sin(euly), sz = sin(eulz);
    Mat rotm;
    rotm = (Mat_<double>(3,3) << cy*cz, sy*sx*cz-sz*cx, sy*cx*cz+sz*sx,
                                            cy*sz, sy*sx*sz+cz*cx, sy*cx*sz-cz*sx,
                                            -sy,          cy*sx,          cy*cx);
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
    eul = (Mat_<double>(1,3) << z, y, x);
    return eul;
}

// TODO: implement rest of functions according to efficiency of conversion
