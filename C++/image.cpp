#include "image.h"

Image::Image(Mat image_init) {

    detector = SURF::create(MINHESSIAN);
    extractor = SURF::create();

    image = Mat(HEIGHT, WIDTH, CVTYPE);
    image = image_init;

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