#ifndef MAIN_CPP_CAMERA_H
#define MAIN_CPP_CAMERA_H

#include "defs.h"
#include "image.h"
#include <ueye.h>

class Camera {

    HIDS cameraHandle;
    void SpawnCameraError(const string& where);
    Mat ConvertCharToMat(char *img);

public:
    double intrinsics[3][3];
    vector<double> rad_dist;
    vector<double> tan_dist;

    Camera();
    bool ConnectCamera();
    bool CaptureImage(Mat& img);
    bool CaptureImage(Image&);
    bool CalibrateCamera();
    bool DisconnectCamera();
};

void ShowImage(Mat imgMatrix);

#endif
