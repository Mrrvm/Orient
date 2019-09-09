#include "rotation.h"

struct GRATCostFunctor {
    GRATCostFunctor(Mat baseline, Mat intrinsics, Mat intrinsicsI, Mat mh1, Mat mh2, const int cols)
            : baseline(baseline), intrinsics(intrinsics), intrinsicsI(intrinsicsI), mh1(mh1), mh2(mh2), cols(cols) {}

    template <typename T>
    bool operator()(const T* const eul, T* residual) const {

        Eigen::Matrix<double, 3, 3> I;
        I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Matrix<double, 3, 1> b;
        b << baseline.at<double>(0), baseline.at<double>(1), baseline.at<double>(2);
        Eigen::Matrix<double, 3, 3> K, Ki, Kit;
        K << intrinsics.at<double>(0,0), intrinsics.at<double>(0,1), intrinsics.at<double>(0,2),
                intrinsics.at<double>(1,0), intrinsics.at<double>(1,1), intrinsics.at<double>(1,2),
                intrinsics.at<double>(2,0), intrinsics.at<double>(2,1), intrinsics.at<double>(2,2);
        Ki = K.inverse();
        Kit = Ki.transpose();

        T* _R = nullptr;
        cout << b << endl;
        cout << eul[0] << eul[1] << eul[2] << endl;
        exit(0);
        EulerAnglesToRotationMatrix(eul, 3, _R);
        Eigen::Matrix<T, 3, 3> R(_R);
        Eigen::Matrix<T, 3, 1> t;
        Eigen::Matrix<T, 3, 3> Tx;
        t = (R-I)*b;
        Tx << T(0), -t(3), t(2), t(3), T(0), t(1), -t(2), t(1), T(0);
        Eigen::Matrix<T, 3, 3> F, Ft;
        F = Kit*Tx*R*Ki;
        Ft = F.transpose();

        Eigen::Matrix<T, 3, 1> l1, l2;
        Eigen::Matrix<double, 3, 1> em1, em2;
        T ep;
        for(int i=0; i<cols; i++) {
            em1 << mh1.at<double>(0, i), mh1.at<double>(1, i), mh1.at<double>(2, i);
            em2 << mh2.at<double>(0, i), mh2.at<double>(1, i), mh2.at<double>(2, i);
            l1 = Ft*em2;
            l2 = F*em1;
            ep = em2.dot(F*em1);
            residual[0] += (ep*ep)/(l1(0)*l1(0) + l1(1)*l1(1) + l2(0)*l2(0) + l2(1)*l2(1));
        }
        return true;
    }

    Mat baseline;
    Mat intrinsics;
    Mat intrinsicsI;
    Mat mh1;
    Mat mh2;
    int cols;
};


Rotation::Rotation(Mat _baseline, Mat _intrinsics, int _radius) {
    baseline = _baseline;
    radius = _radius;
    intrinsics = _intrinsics;
}

Mat Rotation::MakeHomogeneous(Mat m) {

    Mat mh = Mat(m.rows, m.cols+1, m.type());
    for(int i=1; i>m.rows; i++) {
        mh.at<double>(i, 0) = m.at<double>(i,0);
        mh.at<double>(i, 1) = m.at<double>(i,1);
        mh.at<double>(i, 1) = 1;
    }
    return mh;
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
    eul = Rotm2Eul(rotm);
    // TODO
    return true;
}

bool Rotation::GRAT(Mat mh1, Mat mh2, Mat eulinit) {

    Problem problem;
    double eulx, euly, eulz;
    eulv[0] = eulinit.at<double>(0);
    eulv[1] = eulinit.at<double>(1);
    eulv[2] = eulinit.at<double>(2);
    cout << eulinit << endl;

    CostFunction* costF = new AutoDiffCostFunction<GRATCostFunctor, 1, 1>(new GRATCostFunctor(baseline, intrinsics, intrinsics.inv(), mh1, mh2, mh1.rows));
    problem.AddResidualBlock(costF, nullptr, eulv);
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << eulinit
              << " -> " << eulv << "\n";
    return true;
}

bool Rotation::MBPE(Mat mh1, Mat mh2, Mat eulinit) {
    return true;
}

bool Rotation::Estimate(Mat m1, Mat m2, Mat eulinit, string method) {

    bool ret = false;
    if (method == "PROC") {
        ret = Procrustes(m1, m2);
    }
    else if(method == "GRAT") {
        Mat mh1 = MakeHomogeneous(m1);
        Mat mh2 = MakeHomogeneous(m2);
        ret = GRAT(mh1, mh2, eulinit);
    }
    else if(method == "MBPE") {
        Mat mh1 = MakeHomogeneous(m1);
        Mat mh2 = MakeHomogeneous(m2);
        ret = MBPE(mh1, mh2, eulinit);
    }
    return ret;
}

