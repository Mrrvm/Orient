#include "rotation.h"

struct GRATCostFunctor {
    GRATCostFunctor(Mat baseline, Mat intrinsics, Mat mh1, Mat mh2, const int cols)
            : baseline(baseline), intrinsics(intrinsics), mh1(mh1), mh2(mh2), cols(cols) {}

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

        const T eulx = eul[2];
        const T euly = eul[1];
        const T eulz = eul[0];
        T cx = cos(eulx), cy = cos(euly), cz = cos(eulz);
        T sx = sin(eulx), sy = sin(euly), sz = sin(eulz);
        Eigen::Matrix<T, 3, 3> R;
        R << cy*cz, sy*sx*cz-sz*cx, sy*cx*cz+sz*sx,
             cy*sz, sy*sx*sz+cz*cx, sy*cx*sz-cz*sx,
               -sy,          cy*sx,          cy*cx;

        Eigen::Matrix<T, 3, 1> t;
        Eigen::Matrix<T, 3, 3> Tx;
        t = (R-I)*b;
        Tx << T(0), -t(2), t(1), t(2), T(0), -t(0), -t(1), t(0), T(0);
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
    Mat mh1;
    Mat mh2;
    int cols;
};

struct MBPECostFunctor {
    MBPECostFunctor(Mat baseline, Mat intrinsics, Mat mh1, Mat mh2, const int cols)
            : baseline(baseline), intrinsics(intrinsics), mh1(mh1), mh2(mh2), cols(cols) {}

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

        const T eulx = parameters[0][2];
        const T euly = parameters[0][1];
        const T eulz = parameters[0][0];

        T cx = cos(eulx), cy = cos(euly), cz = cos(eulz);
        T sx = sin(eulx), sy = sin(euly), sz = sin(eulz);
        Eigen::Matrix<T, 3, 3> R, Rt;
        R << cy*cz, sy*sx*cz-sz*cx, sy*cx*cz+sz*sx,
                cy*sz, sy*sx*sz+cz*cx, sy*cx*sz-cz*sx,
                -sy,          cy*sx,          cy*cx;
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
        M.col(i) = lambda*km;
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

bool Rotation::Procrustes(Mat M1, Mat M2) {

    if(M1.cols < 3) {
        return false;
    }

    Mat A = M1*M2.t();
    Mat U, s, Vt;
    SVDecomp(A, s, U, Vt);
    Mat rota = Vt.t()*U.t();
    rotm = rota.t();
    eul = Rotm2Eul(rotm);
    //quat = Rotm2Quat(rotm);
    //rotv = Rotm2Rotv(rotm);
    return true;
}


bool Rotation::GRAT(Mat mh1, Mat mh2, Mat eulinit, bool info) {

    double eulv[3];
    eulv[0] = eulinit.at<double>(0);
    eulv[1] = eulinit.at<double>(1);
    eulv[2] = eulinit.at<double>(2);

    Problem problem;
    CostFunction* costF = new AutoDiffCostFunction<GRATCostFunctor, 1, 3>(
            new GRATCostFunctor(baseline, intrinsics, mh1, mh2, mh1.cols));
    problem.AddResidualBlock(costF, nullptr, eulv);
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = info;
    Solver::Summary solverSummary;
    ceres::Solve(options, &problem, &solverSummary);

    if(info)
        cout << solverSummary.BriefReport() << "\n";

    eul.at<double>(0) = eulv[0];
    eul.at<double>(1) = eulv[1];
    eul.at<double>(2) = eulv[2];

    return true;
    // TODO : frees
}

bool Rotation::MBPE(Mat mh1, Mat mh2, Mat eulinit, Mat depthinit, bool info) {

    vector<double*> parameters(2);
    parameters.at(0) = new double[3];
    parameters.at(1) = new double[depthinit.cols];
    parameters.at(0)[0] = eulinit.at<double>(0);
    parameters.at(0)[1] = eulinit.at<double>(1);
    parameters.at(0)[2] = eulinit.at<double>(2);
    for(int i = 0; i < depthinit.cols; i++) {
        parameters.at(1)[i] = depthinit.at<double>(i);
    }

    Problem problem;
    DynamicAutoDiffCostFunction<MBPECostFunctor, 4>* costF =
            new DynamicAutoDiffCostFunction<MBPECostFunctor, 4>(
                new MBPECostFunctor(baseline, intrinsics, mh1, mh2, mh1.cols));
    costF->AddParameterBlock(3);
    costF->AddParameterBlock(depthinit.cols);
    costF->SetNumResiduals(1);
    problem.AddResidualBlock(costF, nullptr, parameters);
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = info;
    Solver::Summary solverSummary;
    ceres::Solve(options, &problem, &solverSummary);

    if(info)
        cout << solverSummary.BriefReport() << "\n";

    eul.at<double>(0) = parameters.at(0)[0];
    eul.at<double>(1) = parameters.at(0)[1];
    eul.at<double>(2) = parameters.at(0)[2];

    return true;
    // TODO : frees
}

bool Rotation::Estimate(Mat m1, Mat m2, string method, Mat eulinit, bool info) {

    bool ret = false;
    if (method == "PROC") {
        Mat M1 = ProjectToSphere(m1);
        Mat M2 = ProjectToSphere(m2);
        ret = Procrustes(M1, M2);
    }
    else if(method == "GRAT") {
        Mat mh1 = MakeHomogeneous(m1);
        Mat mh2 = MakeHomogeneous(m2);
        ret = GRAT(mh1, mh2, eulinit, info);
    }
    else if(method == "MBPE") {
        Mat mh1 = MakeHomogeneous(m1);
        Mat mh2 = MakeHomogeneous(m2);
        Mat M1 = ProjectToSphere(m1);
        ret = MBPE(mh1, mh2, eulinit, M1.row(2), info);
    }
    return ret;
}

double Rotation::ComputeError(Mat rgt) {
    Mat diff = rgt-eul;
    return sqrt(diff.dot(diff));
}

int Rotation::TestR(vector<int>& inliers, vector<Point3d>& M1i, vector<Point3d>& M2i, vector<Point3d> M1r, vector<Point3d> M2r, double maxErr) {

    int cols = M1r.size();
    int j = 0;
    int score = 0;
    double err;
    Mat V1, V2, diff;

    for(int i = 0; i < cols; i++) {
        V1 = Mat(M1r.at(i)).reshape(1, 3);
        V2 = Mat(M2r.at(i)).reshape(1, 3);
        diff = V2 - rotm*V1;
        err = (diff.at<double>(0)+diff.at<double>(1)+diff.at<double>(2))/3;
        while(inliers.at(j) != 0) j++;
        if(abs(err) < maxErr) {
            inliers.at(j) = 1;
            M1i.push_back(M1r.at(i));
            M2i.push_back(M2r.at(i));
            score++;
        }
        j++;
    }
    return score;
}

bool Rotation::RansacByProcrustes(Mat& m1, Mat& m2, int minMatches, double maxErr) {
    vector<DMatch> matches;
    return RansacByProcrustes(m1, m2, matches, minMatches, maxErr);
}

bool Rotation::RansacByProcrustes(Mat& m1, Mat& m2, vector<DMatch>& matches, int minMatches, double maxErr) {
    int i = 0, j = 0, z = 0;
    Mat M1, M2;
    vector<Point3d> M1i, M2i, M1r, M2r;
    Mat M1aux(3, minMatches, DataType<double>::type), M2aux(3, minMatches, DataType<double>::type);
    Mat bestR, maybeR;
    int score = 0, bestScore = -1;
    vector<int> inliers(m1.cols, 0), bestInliers(m1.cols, 0);
    int done = 0;
    Mat A, U, s, Vt;
    bool ret;

    if(m1.cols < minMatches)
        return false;

    M1 = ProjectToSphere(m1);
    M2 = ProjectToSphere(m2);

    int goodMatches = round(MAXMATCHES*0.5);
    int maxIter = round(log(1-0.99)/log(1-pow(1-OUTLIERPER, MINMATCHES)));
    ret = Procrustes(M1, M2);
    if(!ret) return false;
    bestR = rotm;
    srand(time(0));

    for(i = 0; i < maxIter; i++) {

        done = 0;
        while(done != minMatches){
            inliers.at(rand()%(m1.cols)) = 1;
            done = 0;
            for(j = 0; j < m1.cols; j++) done += inliers.at(j);
        }

        z = 0;
        for(j = 0; j < m1.cols; j++){
            if(inliers.at(j)) {
                M1i.push_back(Point3d(M1.at<double>(0, j), M1.at<double>(1, j), M1.at<double>(2, j)));
                M2i.push_back(Point3d(M2.at<double>(0, j), M2.at<double>(1, j), M2.at<double>(2, j)));
                M1aux.at<double>(0, z) = M1.at<double>(0, j);
                M1aux.at<double>(1, z) = M1.at<double>(1, j);
                M1aux.at<double>(2, z) = M1.at<double>(2, j);
                M2aux.at<double>(0, z) = M2.at<double>(0, j);
                M2aux.at<double>(1, z) = M2.at<double>(1, j);
                M2aux.at<double>(2, z) = M2.at<double>(2, j);
                z++;
            }
            else {
                M1r.push_back(Point3d(M1.at<double>(0, j), M1.at<double>(1, j), M1.at<double>(2, j)));
                M2r.push_back(Point3d(M2.at<double>(0, j), M2.at<double>(1, j), M2.at<double>(2, j)));
            }
        }

        ret = Procrustes(M1aux, M2aux);
        if(!ret) return false;

        score = TestR(inliers, M1i, M2i, M1r, M2r, maxErr) + minMatches;
        if(score >= goodMatches) {
            if(score > bestScore) {
                bestR = rotm;
                bestScore = score;
                bestInliers = inliers;
            }
        }

        M1i.clear();
        M2i.clear();
        M1r.clear();
        M2r.clear();
        fill(inliers.begin(), inliers.end(), 0);

    }

    Mat m1p, m2p;
    vector<DMatch> posmatches;
    int n = 0;
    i = 0; j = 0;

    if(bestScore > MAXMATCHES) {
        n = MAXMATCHES;
    }
    else if(bestScore > minMatches) {
        n = bestScore;
    }
    else {
        return false;
    }

    m1p.create(2, n, DataType<double>::type);
    m2p.create(2, n, DataType<double>::type);

    while(i < n){
        if(bestInliers.at(j)) {
            m1p.at<double>(0, i) = m1.at<double>(0, j);
            m1p.at<double>(1, i) = m1.at<double>(1, j);
            m2p.at<double>(0, i) = m2.at<double>(0, j);
            m2p.at<double>(1, i) = m2.at<double>(1, j);
            if(!matches.empty())
                posmatches.push_back(matches.at(j));
            i++;
        }
        j++;
    }

    m1.release(); m2.release();
    m1 = m1p; m2 = m2p;
    if(!matches.empty()) {
        matches.clear();
        matches = posmatches;
    }
    return true;
}

