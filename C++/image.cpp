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


bool Image::SavePNG(string path) {
    return imwrite(path+name+".png", image);
}

void Image::Show() {

    namedWindow("Image", 1);
    imshow("Image", image);
    waitKey();
}

bool Image::FindKeypoints() {

    detector->detect(image, keypoints);
    extractor->compute(image, keypoints, descriptors);
}

bool Image::FindMatches(Image img1, Image img2, Mat m1, Mat m2) {

    FlannBasedMatcher matcher;
    vector<DMatch> matches, good_matches;

    matcher.match(img1.descriptors, img2.descriptors, matches);

    // Ransac

}