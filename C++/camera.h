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
    Mat intrinsics;
    Mat radDist;
    Mat tanDist;

    Camera();
    bool Connect();
    bool Capture(Mat& img);
    bool Capture(Image&);
    bool Calibrate();
    bool Disconnect();
};

void ShowImage(Mat imgMatrix);

#endif
