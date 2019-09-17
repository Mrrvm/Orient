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
        K <<    intrinsics.at<double>(0,0), intrinsics.at<double>(0,1), intrinsics.at<double>(0,2),
                intrinsics.at<double>(1,0), intrinsics.at<double>(1,1), intrinsics.at<double>(1,2),
                intrinsics.at<double>(2,0), intrinsics.at<double>(2,1), intrinsics.at<double>(2,2);
        Ki = K.inverse();
        Kit = Ki.transpose();

        const T eulx = eul[0];
        const T euly = eul[1];
        const T eulz = eul[2];

        Eigen::Matrix<T, 3, 3> Rx, Ry, Rz;
        Rx <<   T(1),       T(0),       T(0),
                T(0),       cos(eulx),  -sin(eulx),
                T(0),       sin(eulx),  cos(eulx);
        Ry <<   cos(euly),   T(0),      sin(euly),
                T(0),        T(1),      T(0),
                -sin(euly),  T(0),      cos(euly);
        Rz <<   cos(eulz),    -sin(eulz),  T(0),
                sin(eulz),    cos(eulz),   T(0),
                T(0),         T(0),        T(1);
        Eigen::Matrix<T, 3, 3> R;
        R = Rz * Ry * Rx;

        Eigen::Matrix<T, 3, 1> t;
        Eigen::Matrix<T, 3, 3> Tx;
        t = (R-I)*b;
        Tx << T(0), -t(2), t(1), t(2), T(0), t(0), -t(1), t(0), T(0);
        Eigen::Matrix<T, 3, 3> F, Ft;
        F = Kit*Tx*R*Ki;
        Ft = F.transpose();

        Eigen::Matrix<T, 3, 1> l1, l2;
        Eigen::Matrix<double, 3, 1> em1, em2;
        T ep;
        residual[0] = T(0);

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

struct MBPECostFunctor {
    MBPECostFunctor(Mat baseline, Mat intrinsics, Mat intrinsicsI, Mat mh1, Mat mh2, const int cols)
            : baseline(baseline), intrinsics(intrinsics), intrinsicsI(intrinsicsI), mh1(mh1), mh2(mh2), cols(cols) {}

    template <typename T>
    bool operator()(T const* const* parameters, T* residual) const {

        Eigen::Matrix<double, 3, 3> I;
        I << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        Eigen::Matrix<double, 3, 1> b;
        b << baseline.at<double>(0), baseline.at<double>(1), baseline.at<double>(2);

        Eigen::Matrix<double, 3, 3> K, Ki, Kit;
        K <<    intrinsics.at<double>(0,0), intrinsics.at<double>(0,1), intrinsics.at<double>(0,2),
                intrinsics.at<double>(1,0), intrinsics.at<double>(1,1), intrinsics.at<double>(1,2),
                intrinsics.at<double>(2,0), intrinsics.at<double>(2,1), intrinsics.at<double>(2,2);
        Ki = K.inverse();
        Kit = Ki.transpose();

        const T eulx = parameters[0][0];
        const T euly = parameters[0][1];
        const T eulz = parameters[0][2];

        Eigen::Matrix<T, 3, 3> Rx, Ry, Rz;
        Rx <<   T(1),       T(0),       T(0),
                T(0),       cos(eulx),  -sin(eulx),
                T(0),       sin(eulx),  cos(eulx);
        Ry <<   cos(euly),   T(0),      sin(euly),
                T(0),        T(1),      T(0),
                -sin(euly),  T(0),      cos(euly);
        Rz <<   cos(eulz),    -sin(eulz),  T(0),
                sin(eulz),    cos(eulz),   T(0),
                T(0),         T(0),        T(1);
        Eigen::Matrix<T, 3, 3> R, Rt;
        R = Rz * Ry * Rx;
        Rt = R.transpose();

        Eigen::Matrix<T, 3, 1> t;
        t = (R-I)*b;

        Eigen::Matrix<T, 3, 1> M1, M2;
        Eigen::Matrix<T, 3, 1> em1, em2;
        Eigen::Matrix<double, 3, 1> rm1, rm2;
        T z1, z2, u1, v1, u2, v2;
        residual[0] = T(0);

        for(int i=0; i<cols; i++) {
            rm1 << mh1.at<double>(0, i), mh1.at<double>(1, i), mh1.at<double>(2, i);
            rm2 << mh2.at<double>(0, i), mh2.at<double>(1, i), mh2.at<double>(2, i);
            z1 = parameters[1][i];
            M2 = R*(z1*(Ki*rm1))+t;
            em2 = K*(M2/M2(2));
            z2 = M2(2);
            M1 = Rt*(z2*(Ki*rm2))-Rt*t;
            em1 = K*(M1/M1(2));
            u1 = em1(0) - rm1(0);
            v1 = em1(1) - rm1(1);
            u2 = em2(0) - rm2(0);
            v2 = em2(1) - rm2(1);
            residual[0] += u1*u1 + u2*u2 + v1*v1 + v2*v2;

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

    Mat mh = Mat(m.rows+1, m.cols, m.type());
    for(int i = 0; i < m.cols; i++) {
        mh.at<double>(0, i) = m.at<double>(0, i);
        mh.at<double>(1, i) = m.at<double>(1, i);
        mh.at<double>(2, i) = 1;
    }
    return mh;
}

Mat Rotation::ProjectToSphere(Mat m) {

    double lambda;
    Mat hm = Mat(3, 1, DataType<double>::type);
    Mat km = Mat(3, 1, DataType<double>::type);
    Mat M = Mat(3, m.cols, DataType<double>::type);
    Mat Ki = intrinsics.inv();
    hm.at<double>(2) = 1;

    for(int i = 0; i < m.cols; i ++) {
        hm.at<double>(0) = m.at<double>(0, i);
        hm.at<double>(1) = m.at<double>(1, i);
        km = Ki*hm;
        lambda = radius/sqrt(km.dot(km));
        M.col(i) = lambda*km.t();
    }
    return M;
}

Mat Rotation::ProjectToPlane(Mat M) {
    Mat hm = Mat(3, 1, DataType<double>::type);
    Mat km = Mat(3, 1, DataType<double>::type);
    Mat m = Mat(2, M.cols, DataType<double>::type);
    hm.at<double>(2) = 1;
    for(int i = 0; i < M.cols; i ++) {
        hm.at<double>(0) = M.at<double>(0, i)/M.at<double>(2, i);
        hm.at<double>(1) = M.at<double>(1, i)/M.at<double>(2, i);
        km = intrinsics*hm;
        m.at<double>(0, i) = km.at<double>(0);
        m.at<double>(1, i) = km.at<double>(1);
    }
    return m;
}

bool Rotation::Procrustes(Mat m1, Mat m2) {

    if(m1.cols < 3)
        return false;

    Mat M1 = ProjectToSphere(m1);
    Mat M2 = ProjectToSphere(m2);

    Mat A = M1*M2.t();
    Mat U, s, Vt;
    SVDecomp(A, s, U, Vt);
    rotm = U * Vt;
    eul = Rotm2Eul(rotm);
    // TODO
    return true;
}

bool Rotation::GRAT(Mat mh1, Mat mh2, Mat eulinit) {

    double eul[3];
    eul[0] = eulinit.at<double>(0);
    eul[1] = eulinit.at<double>(1);
    eul[2] = eulinit.at<double>(2);

    Problem problem;
    CostFunction* costF = new AutoDiffCostFunction<GRATCostFunctor, 1, 3>(
            new GRATCostFunctor(baseline, intrinsics, intrinsics.inv(), mh1, mh2, mh1.cols));
    problem.AddResidualBlock(costF, nullptr, eul);
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << "\n";
    return true;
    // TODO : frees
}

bool Rotation::MBPE(Mat mh1, Mat mh2, Mat eulinit, Mat depthinit) {

    vector<double*> parameters(2);
    parameters.at(0) = new double[3];
    parameters.at(1) = new double[depthinit.cols];
    parameters.at(0)[0] = eulinit.at<double>(0);
    parameters.at(0)[1]= eulinit.at<double>(1);
    parameters.at(0)[2] = eulinit.at<double>(2);
    for(int i = 0; i < depthinit.cols; i++) {
        parameters.at(1)[i] = depthinit.at<double>(i);
    }

    Problem problem;
    DynamicAutoDiffCostFunction<MBPECostFunctor, 4>* costF =
            new DynamicAutoDiffCostFunction<MBPECostFunctor, 4>(
                new MBPECostFunctor(baseline, intrinsics, intrinsics.inv(), mh1, mh2, mh1.cols));
    costF->AddParameterBlock(3);
    costF->AddParameterBlock(depthinit.cols);
    costF->SetNumResiduals(1);
    problem.AddResidualBlock(costF, nullptr, parameters);
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << "\n";
    return true;
    // TODO : frees
}

bool Rotation::Estimate(Mat m1, Mat m2, string method, Mat eulinit, Mat depthinit) {

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
        ret = MBPE(mh1, mh2, eulinit, depthinit);
    }
    return ret;
}

