#include "rotation.h"

Rotation::Rotation() {}


Rotation::Rotation(Mat cvbaseline, Mat cvintrinsics, int _radius) {
    cv2eigen(cvbaseline, baseline);
    cv2eigen(cvintrinsics, intrinsics);
    radius = _radius;
}

Mat Rotation::ProjectToSphere(Mat cvm) {
    MatrixXd m, M;
    Mat cvM;
    cv2eigen(cvm, m);
    M = ProjectToSphere(m);
    eigen2cv(M, cvM);
    return cvM;
}

MatrixXd Rotation::ProjectToSphere(MatrixXd m) {

    MatrixXd mh;
    mh << m, MatrixXd::Ones(m.rows(), 1);

    MatrixXd mhk = intrinsics.inverse()*mh;
    ArrayXd aux = mhk.row(1)*mhk.row(1) + mhk.row(2)*mhk.row(2) + MatrixXd::Ones(m.rows(), 1);
    MatrixXd lambda = radius*aux.rsqrt();
    MatrixXd M = lambda*mhk;
    return M;
}


bool Rotation::Procrustes(Mat cvm1, Mat cvm2) {
    MatrixXd m1, m2;
    cv2eigen(cvm1, m1);
    cv2eigen(cvm2, m2);
    return Procrustes(m1, m2);
}


bool Rotation::Procrustes(MatrixXd m1, MatrixXd m2) {
    MatrixXd M1 = ProjectToSphere(m1);
    MatrixXd M2 = ProjectToSphere(m2);

    if(m1.rows() < 3)
        return false;

    Matrix3d A = M1*M2.transpose();
    JacobiSVD<MatrixXd> svd(A, ComputeFullV | ComputeFullU );
    rotm = svd.matrixV()*svd.matrixU().transpose();
    quat = rotm;
    eul = rotm.eulerAngles(2, 1, 0);
    return true;
}

bool Rotation::GRAT() {
    return true;
}

bool Rotation::MBPE() {
    return true;
}

bool Rotation::Estimate(Mat _m1, Mat _m2, string method) {
    MatrixXd m1, m2;
    cv2eigen(_m1, m1);
    cv2eigen(_m2, m2);
    return Estimate(m1, m2, method);
}


bool Rotation::Estimate(MatrixXd m1, MatrixXd m2, string method) {

    if (method == "PROC") {
        Procrustes(m1, m2);
    }
    else if(method == "GRAT") {

    }
    else if(method == "MBPE") {

    }

    return true;
}
