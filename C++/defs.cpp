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

Mat Quat2Eul(Mat q) {

    Mat eul(1, 3, DataType<double>::type);

    double sinr_cosp = 2.0 * (q.at<double>(0)*q.at<double>(1) + q.at<double>(2)*q.at<double>(3));
    double cosr_cosp = 1.0 - 2.0 * (q.at<double>(1)*q.at<double>(1) + q.at<double>(2)*q.at<double>(2));

    eul.at<double>(2) = atan2(sinr_cosp, cosr_cosp);
    double sinp = 2.0 * (q.at<double>(0)*q.at<double>(2) - q.at<double>(3)*q.at<double>(1));

    if (fabs(sinp) >= 1)
        eul.at<double>(1) = copysign(M_PI / 2, sinp);
    else
        eul.at<double>(1) = asin(sinp);

    double siny_cosp = 2.0 * (q.at<double>(0)*q.at<double>(3) + q.at<double>(1)*q.at<double>(2));
    double cosy_cosp = 1.0 - 2.0 * (q.at<double>(2)*q.at<double>(2) + q.at<double>(3)*q.at<double>(3));
    eul.at<double>(0) = atan2(siny_cosp, cosy_cosp);

    return eul;
}

Mat QuatConjugate(Mat quat) {

    Mat quatc(1, 4, DataType<double>::type);;

    quatc.at<double>(0) = quat.at<double>(0);
    quatc.at<double>(1) = -quat.at<double>(1);
    quatc.at<double>(2) = -quat.at<double>(2);
    quatc.at<double>(3) = -quat.at<double>(3);

    return quatc;
}

Mat QuatMultiply(Mat quat1, Mat quat2) {

    double a1 = quat1.at<double>(0);
    double b1 = quat1.at<double>(1);
    double c1 = quat1.at<double>(2);
    double d1 = quat1.at<double>(3);

    double a2 = quat2.at<double>(0);
    double b2 = quat2.at<double>(1);
    double c2 = quat2.at<double>(2);
    double d2 = quat2.at<double>(3);
    Mat quatr(1, 4, DataType<double>::type);;

    quatr.at<double>(0) = a1*a2 - b1*b2 - c1*c2 - d1*d2;
    quatr.at<double>(1) = a1*b2 + b1*a2 + c1*d2 - d1*c2;
    quatr.at<double>(2) = a1*c2 - b1*d2 + c1*a2 + d1*b2;
    quatr.at<double>(3) = a1*d2 + b1*c2 - c1*b2 + d1*a2;

    return quatr;

}