#include "image.h"

using namespace std;
using namespace cv;

Class Image(Mat image_init) {

    Ptr<SURF> detector = SURF::create(MINHESSIAN);
    Ptr<SURF> extractor = SURF::create();

public:
    Mat image(HEIGHT, WIDTH, CVTYPE) = image_init;
    vector<KeyPoint> keypoints;
    Mat descriptors;
};

void Image::Show() {

    namedWindow("Image", 1);
    imshow("Image", image);
    waitKey();
}

int Image::FindKeypoints() {

    detector->detect(image, keypoints);
    extractor->compute(image, keypoints, descriptors);
}

int findMatches(Image img1, Image img2, int *ind1, int *ind2) {

    FlannBasedMatcher matcher;
    vector<DMatch> matches, good_matches;

    matcher.match(img1.descriptors, img2.descriptors, matches);

    // Ransac

}