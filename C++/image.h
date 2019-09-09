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
    string path;
    string ext;
    vector<KeyPoint> keypoints;
    Mat descriptors;

    Image();
    Image(string, string, string);
    Image(string, string, string, int);
    bool Load();
    bool Save();
    void Show();
    bool FindKeypoints();
    static bool FindMatches(Image, Image, Mat&, Mat&, vector<DMatch>&);
    static void ShowMatches(Image, Image, vector<DMatch>);
    static bool RansacByProcrustes();
    bool DetectChessboard(Image, Image, Mat&, Mat&, Mat&);
};

#endif
