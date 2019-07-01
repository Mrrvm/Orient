#include "camera.h"
#include "defs.h"

int main() {

    HIDS cameraHandle = (HIDS)0;
    if(!ConnectCamera(&cameraHandle))
        SpawnCameraError(cameraHandle, "ConnectCamera");
    Mat imgMatrix = CaptureImage(cameraHandle);

    /*
    Mat imgMatrix = imread(string(IMGSPATH)+"img1_30v.jpg", CV_LOAD_IMAGE_COLOR);
    if(!imgMatrix.data)
        SpawnError("imread");
    ShowImage(imgMatrix);
*/

    is_ExitCamera(cameraHandle);
    return 0;
}