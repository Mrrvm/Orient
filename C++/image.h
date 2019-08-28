#ifndef MAIN_CPP_IMAGE_H
#define MAIN_CPP_IMAGE_H

#include "defs.h"

using namespace xfeatures2d;

class Image {

    Ptr<SURF> detector;
    Ptr<SURF> extractor;

public:
    Mat image;
    vector<KeyPoint> keypoints;
    Mat descriptors;

    Image(Mat);
    void Show();
    int FindKeypoints();

};

int findMatches(Image img1, Image img2, int *ind1, int *ind2);

#endif
