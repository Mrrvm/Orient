#include "camera.h"
#include "image.h"
#include "defs.h"

int main() {

    Camera myCam;
    Image myImage("test");
    bool ret;

    ret = myCam.ConnectCamera();
    if (!ret) cout << "ERROR: Camera not connected" << endl;
    cout << "STATUS: Camera connected" << endl;

    ret = myCam.CaptureImage(myImage);
    if (!ret) cout << "ERROR: Camera did not capture image" << endl;
    cout << "STATUS: Camera captured image" << endl;

    //myImage.Show();

    ret = myImage.Save("~/", "png");
    if (!ret) cout << "ERROR: Image did not save" << endl;
    cout << "STATUS: Image saved" << endl;

    ret = myCam.DisconnectCamera();
    if (!ret)cout << "ERROR: Camera did not disconnect" << endl;
    cout << "STATUS: Camera disconnected" << endl;

    return 0;
}