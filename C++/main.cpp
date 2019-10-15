#include "camera.h"
#include "image.h"
#include "sensor.h"
#include "defs.h"

#define online 0

void ThrowError(string what) {
    cout << RED << "(main.cpp) ERROR : " << RESET << what << endl;
    exit(EXIT_FAILURE);
}

int main() {

    // Variables
    Sensor mySensor;
    Camera myCam;
    Image img;
    Image chessimg;
    Mat quat;
    bool ret;
    string dir;

    dir = "/home/imarcher/Ex3-Lab-Eye/";
    ofstream out(dir+"IMU/imu.data");

    // Connect sensor
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;

    getchar();

    // Connect camera and sensor
    ret = myCam.Connect();
    if (!ret) ThrowError("Camera not connected");
    cout << YELLOW << "STATUS : " << RESET << "Camera connected" << endl;


    int i = 0;
    while(i < 10) {

        cout << "CHANGE ORIENTATION" << endl;
        cout << "REMOVE CHESSBOARD" << endl;
        getchar();

        // Get image
        ret = myCam.Capture(img);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        img.Show();
        ret = img.Save(to_string(i)+"img", dir+"features/", "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        cout << "PUT CHESSBOARD" << endl;
        getchar();

        // Get chessboard image
        ret = myCam.Capture(chessimg);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        chessimg.Show();
        ret = chessimg.Save(to_string(i)+"chessimg", dir+"GT/", "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        // Get orientation
        ret = mySensor.GetOrientation();
        if (!ret) ThrowError("Sensor was not able to get orientation");
        cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
        mySensor.quat.copyTo(quat);
        cout << BLUE << "IMU (degrees)" << RESET << quat << endl;
        out << quat << endl;

        i++;
    }
    out.close();
    exit(0);
}
