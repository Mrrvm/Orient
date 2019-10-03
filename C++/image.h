#ifndef MAIN_CPP_IMAGE_H
#define MAIN_CPP_IMAGE_H

#include "defs.h"

using namespace xfeatures2d;

class Image {

    Ptr<SURF> detector;
    Ptr<SURF> extractor;
    Mat rgbimage;

public:
    Mat image;
    vector<KeyPoint> keypoints;
    Mat descriptors;

    Image();
    bool Load(string, string, string);
    bool Save(string, string, string);
    void Show();
    void Undistort(Mat, Mat);
    bool FindKeypoints(int hessian = MINHESSIAN);
    static bool FindMatches(Image, Image, Mat&, Mat&);
    static bool FindMatches(Image, Image, Mat&, Mat&, vector<DMatch>&);
    static void ShowMatches(Image, Image, vector<DMatch>);
    bool DetectChessboardRotation(int, int, int, Mat, Mat, Mat&);
};

#endif
