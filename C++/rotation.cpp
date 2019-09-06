#include "rotation.h"

struct GRATCostFunctor {
    template <typename T>
    bool operator()(const T* const eul, T* residual) const {
        cout << eul[0];
        return true;
    }
};

Rotation::Rotation(double* _baseline, Mat _intrinsics, int _radius) {
    baseline = _baseline;
    radius = _radius;
    intrinsics = _intrinsics;
}

Mat Rotation::ProjectToSphere(Mat m) {

    double lambda;
    Mat hm = Mat(3, 1, DataType<double>::type);
    Mat km = Mat(3, 1, DataType<double>::type);
    Mat M = Mat(m.rows, 3, DataType<double>::type);
    Mat Ki = intrinsics.inv();
    hm.at<double>(2) = 1;

    for(int i = 0; i < m.rows; i ++) {
        hm.at<double>(0) = m.at<double>(i,0);
        hm.at<double>(1) = m.at<double>(i,1);
        km = Ki*hm;
        lambda = radius/sqrt(km.dot(km));
        M.row(i) = lambda*km.t();
    }
    return M;
}


bool Rotation::Procrustes(Mat m1, Mat m2) {

    if(m1.rows < 3)
        return false;

    Mat M1 = ProjectToSphere(m1);
    Mat M2 = ProjectToSphere(m2);

    Mat A = M1.t()*M2;
    Mat U, s, Vt;
    SVDecomp(A, s, U, Vt);
    rotm = U * Vt;
    eul = rotm2eul(rotm);
    // TODO
    return true;
}

bool Rotation::GRAT(Mat m1, Mat m2, Mat eulinit) {

    double eulvar[3] = {eulinit.at<double>(0, 0),  eulinit.at<double>(0, 1), eulinit.at<double>(0, 2)};
    Problem problem;

    CostFunction* costF = new AutoDiffCostFunction<GRATCostFunctor, 1, 1>(new GRATCostFunctor);
    problem.AddResidualBlock(costF, nullptr, eulvar);
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << eulinit
              << " -> " << eulvar << "\n";
    return true;
}

bool Rotation::MBPE(Mat m1, Mat m2, Mat eulinit) {
    return true;
}


bool Rotation::Estimate(Mat m1, Mat m2, Mat eulinit, string method) {

    bool ret = false;
    if (method == "PROC") {
        ret = Procrustes(m1, m2);
    }
    else if(method == "GRAT") {
        ret = GRAT(m1, m2, eulinit);
    }
    else if(method == "MBPE") {
        ret = MBPE(m1, m2, eulinit);
    }
    return ret;
}
