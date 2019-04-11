#include "camera.h"

void SpawnCameraError(HIDS cameraHandle, string where) {
    char *errDesc;
    int err = 0;
    is_GetError(cameraHandle, &err, &errDesc);
    cout << "ERROR AT " << where << " : " << errDesc << endl;
    is_ExitCamera(cameraHandle);
    exit(EXIT_FAILURE);
}

int ConnectCamera(HIDS *cameraHandle) {

    int ret = is_InitCamera(cameraHandle, NULL);
    if (ret != IS_SUCCESS) {
        //Check if GigE uEye SE needs a new starter firmware
        if (ret == IS_STARTER_FW_UPLOAD_NEEDED) {
            // Calculate time needed for updating the starter firmware for display
            int time = 0;
            is_GetDuration(*cameraHandle, IS_SE_STARTER_FW_UPLOAD, &time);
            cout << "Updating for " << time << endl;
            // Upload new starter firmware during initialization
            *cameraHandle = *cameraHandle | IS_ALLOW_STARTER_FW_UPLOAD;
            ret = is_InitCamera(cameraHandle, NULL);
            if (ret == IS_SUCCESS)
                return 1;
        }
        return 0;
    }
    return 1;
}

void ShowImage(Mat imgMatrix) {

    namedWindow("Image", 1);
    imshow("Image", imgMatrix);
    waitKey();
}

Mat ConvertCharToMat(char *img)  {
    Mat imgMatrix(HEIGHT, WIDTH, CVTYPE);
    if(COLOR) {
        int j = 0, i = 0, m = 0;
        while (j < HEIGHT) {
            while (i < WIDTH * 4) {
                if (i % 4 != 3) {
                    imgMatrix.ptr(j)[i] = img[j * WIDTH * 4 + m];
                    ++m;
                } else {
                    imgMatrix.ptr(j)[i] = img[j * WIDTH * 4 + i + WIDTH * 3];
                }
                ++i;
            }
            i = 0;
            j++;
            m = 0;
        }
    }
    else {
        memcpy(imgMatrix.data, img, WIDTH*HEIGHT);
    }
    return imgMatrix;
}

Mat CaptureImage(HIDS cameraHandle) {

    char *img = NULL;
    int id = 0;
    Mat imgMatrix(HEIGHT, WIDTH, CVTYPE);

    if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &img, &id) != IS_SUCCESS)
        SpawnCameraError(cameraHandle, "is_AllocImageMem");
    if(is_SetImageMem(cameraHandle, img, id) != IS_SUCCESS)
        SpawnCameraError(cameraHandle, "is_SetImageMem");
    if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS)
        SpawnCameraError(cameraHandle, "is_FreezeVideo");
    imgMatrix = ConvertCharToMat(img);
    cout << "nfpeine" << endl;
    if(is_FreeImageMem(cameraHandle, img, id) != IS_SUCCESS)
        SpawnCameraError(cameraHandle, "is_FreeImageMem");
    return imgMatrix;
}
