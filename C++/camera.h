#ifndef MAIN_CPP_CAMERA_H
#define MAIN_CPP_CAMERA_H

#include "defs.h"
#include <ueye.h>

class Camera {

    HIDS cameraHandle;
    Mat ConvertCharToMat(char *img);

public:
    double intrinsics[3][3];
    vector<double> rad_dist;
    vector<double> tan_dist;

    Camera();
    void SpawnCameraError(const string& where);
    int ConnectCamera();
    int CaptureImage(Mat img);
    int CalibrateCamera();
    int DisconnectCamera();
};

void ShowImage(Mat imgMatrix);

#endif
