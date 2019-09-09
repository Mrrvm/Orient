#include "image.h"

Image::Image() {

    detector = SURF::create(MINHESSIAN);
    extractor = SURF::create();

};

Image::Image(string _name, string _path, string _ext) {

    detector = SURF::create(MINHESSIAN);
    extractor = SURF::create();
    name = _name;
    path = _path;
    ext = _ext;
};

Image::Image(string _name, string _path, string _ext, int hessian) {

    detector = SURF::create(hessian);
    extractor = SURF::create();
    name = _name;
    path = _path;
    ext = _ext;
};


bool Image::Load() {
    image = imread(path + name + "." + ext, IMREAD_GRAYSCALE);
    if(image.empty())
        return false;
    return true;
}

bool Image::Save() {
    return imwrite(path + name + "." + ext, image);
}

void Image::Show() {
    namedWindow("Image", WINDOW_AUTOSIZE);
    imshow("Image", image);
}

bool Image::FindKeypoints() {

    detector->detect(image, keypoints);
    if(keypoints.empty())
        return false;
    extractor->compute(image, keypoints, descriptors);
        return descriptors.rows != 0;
}

bool Image::FindMatches(Image img1, Image img2, Mat& m1, Mat& m2, vector<DMatch>& matches) {

    FlannBasedMatcher matcher;
    vector<Point2d> p1, p2;
    double maxDist = 0, minDist = 100, dist = 0;

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
            if(abs(x1) > 1e-2 && abs(x2) > 1e-2 && abs(y1) > 1e-2 && abs(y2) > 1e-2) {
                p1.emplace_back(x1, y1);
                p2.emplace_back(x2, y2);
            }
        }
    }
    m1 = Mat(p1, DataType<double>::type);
    m2 = Mat(p2, DataType<double>::type);

    return m1.rows >= 3;
}

void Image::ShowMatches(Image img1, Image img2, vector<DMatch> matches) {
    Mat imgOutput;
    drawMatches(img1.image, img1.keypoints, img2.image, img2.keypoints,
                matches, imgOutput, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow(img1.name+" and "+img2.name+" keypoint matches", imgOutput);
}

bool Image::RansacByProcrustes() {
//TODO
}

bool Image::DetectChessboard(Image img1, Image img2, Mat& m1, Mat& m2, Mat& rot) {
//TODO
}