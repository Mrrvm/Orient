#include "camera.h"

Camera::Camera() {
    cameraHandle = (HIDS)0;
};

void Camera::SpawnCameraError(const string& where) {
    char *errDesc;
    int err = 0;
    int ret = is_GetError(cameraHandle, &err, &errDesc);
    if(ret == IS_SUCCESS) {
        cout << RED << "(camera.cpp) ERROR AT " << RESET << where << RED << " : " << RESET << errDesc << endl;
        errDesc = 0;
    }
    else {
        cout << RED << "(camera.cpp) ERROR AT " << RESET << where << RED << " : " << RESET << "unknown" << endl;
    }
}

bool Camera::Connect() {

    int ret = is_InitCamera(&cameraHandle, nullptr);
    if (ret == IS_SUCCESS)
        return true;
    if (ret == IS_STARTER_FW_UPLOAD_NEEDED) {
        int time = 0;
        is_GetDuration(cameraHandle, IS_SE_STARTER_FW_UPLOAD, &time);
        cameraHandle = cameraHandle | IS_ALLOW_STARTER_FW_UPLOAD;
        ret = is_InitCamera(&cameraHandle, nullptr);
        if (ret == IS_SUCCESS)
            return true;
    }
    SpawnCameraError("is_InitCamera");
    return false;

}

Mat Camera::ConvertCharToMat(char *img)  {
    Mat imgMat(HEIGHT, WIDTH, CVTYPE);
    int j = 0, i = 0, m = 0;
    while (j < HEIGHT) {
        while (i < WIDTH * 4) {
            if (i % 4 != 3) {
                imgMat.ptr(j)[i] = img[j * WIDTH * 4 + m];
                ++m;
            } else {
                imgMat.ptr(j)[i] = img[j * WIDTH * 4 + i + WIDTH * 3];
            }
            ++i;
        }
        i = 0;
        j++;
        m = 0;
    }
    return imgMat;
}

bool Camera::Capture(Mat& imgMat) {

    char *img = nullptr;
    int id = 0;

    if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &img, &id) != IS_SUCCESS) {
        SpawnCameraError("is_AllocImageMem");
        return false;
    }
    if(is_SetImageMem(cameraHandle, img, id) != IS_SUCCESS) {
        SpawnCameraError("is_SetImageMem");
        return false;
    }
    if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS) {
        SpawnCameraError("is_FreezeVideo");
        return false;
    }
    if(img == nullptr)
        return false;
    imgMat = ConvertCharToMat(img);
    if(is_FreeImageMem(cameraHandle, img, id) != IS_SUCCESS) {
        SpawnCameraError("is_FreeImageMem");
        return false;
    }
    return true;
}

bool Camera::Capture(Image& imgobj) {

    Mat imgMat;
    bool ret;
    ret = Capture(imgMat);
    if(!ret)
        return false;
    cvtColor(imgMat, imgobj.image, COLOR_BGR2GRAY);
    return true;
}

bool Camera::Calibrate() {
    // Read all images
    // Calibrate with chessboard
    // Return intrinsics and distortion
}

bool Camera::Disconnect() {
    int ret = is_ExitCamera(cameraHandle);
    if(ret != IS_SUCCESS) {
        SpawnCameraError("is_ExitCamera");
        return false;
    }
    return true;
}