#include "image.h"

Image::Image() {};

bool Image::Load(string name, string path, string ext) {
    image = imread(path + name + "." + ext, IMREAD_GRAYSCALE);
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
    double maxDist = 0, minDist = 100, dist = 0;
    Mat pm1, pm2;
    vector<DMatch> pmatches;

    matcher.match(img1.descriptors, img2.descriptors, matches);
    if(matches.empty())
        return false;

    // Ensure a small euclidean distance between matching descriptors
    for(auto & match : matches) {
        dist = match.distance;
        if(dist < minDist) minDist = dist;
        if(dist > maxDist) maxDist = dist;
    }
    double x1, x2, y1, y2;
    for(auto & match : matches) {
        if(match.distance <= max(2*minDist, 0.02)) {
            x1 = img1.keypoints[match.queryIdx].pt.x;
            y1 = img1.keypoints[match.queryIdx].pt.y;
            x2 = img2.keypoints[match.trainIdx].pt.x;
            y2 = img2.keypoints[match.trainIdx].pt.y;
            p1.emplace_back(x1, y1);
            p2.emplace_back(x2, y2);
            pmatches.push_back(match);
        }
    }

    pm1 = pm2 = Mat(2, p1.size(), DataType<double>::type);
    for(int i=0; i < p1.size(); i++) {
        pm1.at<double>(0, i) = p1.at(i).x;
        pm1.at<double>(1, i) = p1.at(i).y;
        pm1.at<double>(0, i) = p1.at(i).x;
        pm1.at<double>(1, i) = p1.at(i).y;
    }

    p1.clear(); p2.clear(); matches.clear();
    matches = pmatches;
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
    vector<vector<Point3f>> objpts;
    vector<vector<Point2f>> imgpts;
    vector<Point3f> obj;
    Mat tr;
    Size boardsz = Size(hcorners, vcorners);
    int numSquares = hcorners*vcorners;
    int ret;

    for(int j=0; j<numSquares; j++)
        obj.push_back(Point3f(j/hcorners, j%hcorners, 0.0f));
    ret = findChessboardCorners(image, boardsz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    if(!ret)
        return false;
    cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, sqlen, 0.1));
    imgpts.push_back(corners);
    objpts.push_back(obj);
    ret = solvePnPRansac(objpts, imgpts, intrinsics, distcoeff, R, tr);
    if(!ret)
        return false;

}