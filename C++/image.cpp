#include "image.h"

Image::Image() {

    detector = SURF::create(MINHESSIAN);
    extractor = SURF::create();

};

Image::Image(string imageName) {

    //detector = SURF::create(MINHESSIAN);
    //extractor = SURF::create();
    name = imageName;
};

bool Image::Save(string path, string ext) {
    cout << path+name+"."+ext << endl;
    return imwrite(path+name+"."+ext, image);
}

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