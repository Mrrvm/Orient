#ifndef MAIN_CPP_IMAGE_H
#define MAIN_CPP_IMAGE_H

#include "defs.h"

using namespace xfeatures2d;

class Image {

    Ptr<SURF> detector;
    Ptr<SURF> extractor;

public:
    Mat image;
    string name;
    vector<KeyPoint> keypoints;
    Mat descriptors;

    Image();
    Image(string);
    Image(string, int);
    bool Save(string, string);
    void Show();
    bool FindKeypoints();
    static bool FindMatches(Image, Image, Mat&, Mat&, vector<DMatch>&);
    static void ShowMatches(Image, Image, vector<DMatch>);
    static bool RansacByProcrustes();
    bool DetectChessboard(Image, Image, Mat&, Mat&, Mat&);
};

#endif
