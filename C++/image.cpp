#include "image.h"

Image::Image() {

    detector = SURF::create(MINHESSIAN);
    extractor = SURF::create();

};

Image::Image(string _name) {

    detector = SURF::create(MINHESSIAN);
    extractor = SURF::create();
    name = _name;
};

Image::Image(string _name, int hessian) {

    detector = SURF::create(hessian);
    extractor = SURF::create();
    name = move(_name);
};

bool Image::Save(string path, string ext) {
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
    m1 = Mat(matches.size(), 2, DataType<double>::type);
    m2 = Mat(matches.size(), 2, DataType<double>::type);
    for(int i = 0; i < int(matches.size()); i++) {
        if(matches[i].distance <= max(2*minDist, 0.02)) {
            m1.at<double>(i, 0) = img1.keypoints[matches[i].queryIdx].pt.x;
            m1.at<double>(i, 1) = img1.keypoints[matches[i].queryIdx].pt.y;
            m2.at<double>(i, 0) = img2.keypoints[matches[i].queryIdx].pt.x;
            m2.at<double>(i, 1) = img2.keypoints[matches[i].queryIdx].pt.y;
        }
    }
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