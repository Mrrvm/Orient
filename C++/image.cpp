#include "image.h"

Image::Image() {};

bool Image::Load(string name, string path, string ext) {
    //Mat rgbimage;
    image = imread(path + name + "." + ext);
    //cvtColor(rgbimage, image, COLOR_BGR2GRAY);
    if(image.empty())
        return false;
    return true;
}

bool Image::Save(string name, string path, string ext) {
    return imwrite(path + name + "." + ext, image);
}

void Image::Show() {
    namedWindow("Image", WINDOW_AUTOSIZE);
    imshow("Image", image);
    waitKey(0);
}

void Image::Undistort(Mat intrinsics, Mat distcoeff) {
    Mat pimg = image;
    image.release();
    undistort(pimg, image, intrinsics, distcoeff);
};

bool Image::FindKeypoints(int hessian) {

    detector = SURF::create(hessian);
    extractor = SURF::create();
    detector->detect(image, keypoints);
    if(keypoints.empty())
        return false;
    extractor->compute(image, keypoints, descriptors);
        return descriptors.rows != 0;
}

bool Image::FindMatches(Image img1, Image img2, Mat& m1, Mat& m2) {
    vector<DMatch> matches;
    return FindMatches(img1, img2, m1, m2, matches);
}

bool Image::FindMatches(Image img1, Image img2, Mat& m1, Mat& m2, vector<DMatch>& matches) {

    FlannBasedMatcher matcher;
    vector<Point2d> p1, p2;
    Mat pm1, pm2;
    vector<DMatch> pmatches;

    matcher.match(img1.descriptors, img2.descriptors, matches);
    if(matches.empty())
        return false;

    double x1, x2, y1, y2;
    for(auto & match : matches) {
        x1 = img1.keypoints[match.queryIdx].pt.x;
        y1 = img1.keypoints[match.queryIdx].pt.y;
        x2 = img2.keypoints[match.trainIdx].pt.x;
        y2 = img2.keypoints[match.trainIdx].pt.y;
        p1.emplace_back(x1, y1);
        p2.emplace_back(x2, y2);
        pmatches.push_back(match);
    }
    pm1 = Mat(2, p1.size(), DataType<double>::type);
    pm2 = Mat(2, p1.size(), DataType<double>::type);
    for(int i=0; i < p1.size(); i++) {
        pm1.at<double>(0, i) = p1.at(i).x;
        pm1.at<double>(1, i) = p1.at(i).y;
        pm2.at<double>(0, i) = p2.at(i).x;
        pm2.at<double>(1, i) = p2.at(i).y;
    }

    p1.clear(); p2.clear(); matches.clear();
    matches = pmatches; //TODO: add an if in case user doesnt use matches
    m1 = pm1;
    m2 = pm2;

    return m1.cols >= 3;
}


void Image::ShowMatches(Image img1, Image img2, vector<DMatch> matches) {
    Mat imgOutput;
    drawMatches(img1.image, img1.keypoints, img2.image, img2.keypoints,
                matches, imgOutput, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow("Keypoint matches", imgOutput);
    waitKey(0);
}

bool Image::DetectChessboardRotation(int hcorners, int vcorners, int sqlen, Mat intrinsics, Mat distcoeff, Mat& R) {

    vector<Point2f> corners;
    vector<Point3f> realpts;
    Mat tr, rot, Rt;
    Size boardsz = Size(hcorners, vcorners);
    int numSquares = hcorners*vcorners;
    int ret;

    for(int j=0; j<numSquares; j++)
        realpts.push_back(Point3f(j/hcorners, j%hcorners, 0.0f));

    ret = findChessboardCorners(image, boardsz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    if(!ret) return false;

    cornerSubPix(image, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, sqlen, 0.1));

    ret = solvePnPRansac(realpts, corners, intrinsics, distcoeff, rot, tr);
    if(!ret) return false;

/*
    //drawChessboardCorners(image, boardsz, corners, ret);
    drawFrameAxes(rgbimage, intrinsics, distcoeff, rot, tr, 5, 5);
    namedWindow("window");
    imshow("window", rgbimage);
    waitKey(0);
*/

    realpts.clear();
    corners.clear();
    Rodrigues(rot, Rt);
    R = Rt.t();
    return true;
}