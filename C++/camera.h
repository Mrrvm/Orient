#ifndef MAIN_CPP_CAPTURE_H
#define MAIN_CPP_CAPTURE_H

#include "defs.h"
#include <ueye.h>

class Camera {

    HIDS cameraHandle;
    Mat ConvertCharToMat(char *img);

public:
    double rotm[3][3];
    vector<double> eul;
    vector<double> tr;
    double intrinsics[3][3];
    vector<double> rad_dist;
    vector<double> tan_dist;

    void SpawnCameraError(string where);
    int ConnectCamera();
    int CaptureImage(Mat img);
    int CalibrateCamera();
    int DisconnectCamera();
};

void ShowImage(Mat imgMatrix);

#endif
