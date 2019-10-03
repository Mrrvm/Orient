#include "../camera.h"
#include "../image.h"
#include "../sensor.h"
#include "../defs.h"

#define online 0

void ThrowError(string what) {
    cout << RED << "(main.cpp) ERROR : " << RESET << what << endl;
    exit(EXIT_FAILURE);
}

int main() {

    // Variables
    Sensor mySensor;
    Camera myCam;
    Image img1, img2;
    Image chessimg1, chessimg2;
    bool ret;
    string dir;

    dir = 'Ex1-Lab/';
    ofstream out("/home/imarcher/"+dir+"imu.data");

    // Connect camera and sensor
    ret = myCam.Connect();
    if (!ret) ThrowError("Camera not connected");
    cout << YELLOW << "STATUS : " << RESET << "Camera connected" << endl;

    getchar();

    // Connect sensor
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;

    getchar();

    int i = 0;
    while(i < 50) {

        // Get image 1
        ret = myCam.Capture(img1);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        img1.Show();
        ret = img1.Save(i+"img1", "/home/imarcher/" + dir, "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        getchar();

        // Get chessboard image 1
        ret = myCam.Capture(chessimg1);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        chessimg1.Show();
        ret = chessimg1.Save(i+"chessimg1", "/home/imarcher/" + dir, "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        getchar();

        // Get orientation 1
        ret = mySensor.GetOrientation();
        if (!ret) ThrowError("Sensor was not able to get orientation");
        cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
        cout << BLUE << "IMU (degrees)" << RESET << mySensor.eul << endl;
        out << mySensor.eul << " , " << mySensor.quat << " , ";

        getchar();

        // Get image 2
        ret = myCam.Capture(img2);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        img2.Show();
        ret = img2.Save(i+"img2", "/home/imarcher/" + dir, "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        getchar();

        // Get chessboard image 2
        ret = myCam.Capture(chessimg2);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        chessimg2.Show();
        ret = chessimg2.Save(i+"chessimg2", "/home/imarcher/" + dir, "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        getchar();

        // Get orientation 2
        ret = mySensor.GetOrientation();
        if (!ret) ThrowError("Sensor was not able to get orientation");
        cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
        cout << BLUE << "IMU (degrees)" << RESET << mySensor.eul << endl;
        out << mySensor.eul << " , " << mySensor.quat << endl;

        i++;
    }
    out.close();
    return 0;
}
