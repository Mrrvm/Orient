#ifndef MAIN_CPP_CAPTURE_H
#define MAIN_CPP_CAPTURE_H

#include "defs.h"

void SpawnCameraError(HIDS cameraHandle, string where);
int ConnectCamera(HIDS *cameraHandle);
void ShowImage(Mat imgMatrix);
Mat CaptureImage(HIDS cameraHandle);


#endif
