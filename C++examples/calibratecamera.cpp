#include "camera.h"
#include "image.h"
#include "sensor.h"
#include "rotation.h"
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
    Image img1, img2;
    Image chessimg1, chessimg2;
    bool ret;
    string dir;

    dir = 'Calib/';

    // Connect camera
    ret = myCam.Connect();
    if (!ret) ThrowError("Camera not connected");
    cout << YELLOW << "STATUS : " << RESET << "Camera connected" << endl;

    getchar();

    int i = 0;
    while(i < 50) {

        // Get image 1
        ret = myCam.Capture(img1);
        if (!ret) ThrowError("Camera did not capture image");
        cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
        img1.Show();
        ret = img1.Save(i+"img", "/home/imarcher/" + dir, "jpg");
        if (!ret) ThrowError("Image did not save");
        cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

        getchar();

        i++;
    }

}
