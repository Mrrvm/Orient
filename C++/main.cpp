#include "camera.h"
#include "image.h"
#include "sensor.h"
#include "rotation.h"
#include "defs.h"

int main() {

    Sensor mySensor;
    Camera myCam;
    Image img1("img1"), img2("img2");
    Rotation myRot;
    Mat ori1, ori2;
    Mat baseline, intrinsics, ranDist, tanDist;
    Mat m1, m2;
    vector<DMatch> matches;
    bool ret;

    // Connect camera and sensor
    ret = myCam.Connect();
    if (!ret) cout << "ERROR: Camera not connected" << endl;
    cout << "STATUS: Camera connected" << endl;
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) cout << "ERROR: Sensor not connected" << endl;
    cout << "STATUS: Sensor connected" << endl;

    // Get data
    ret = myCam.Capture(img1);
    if (!ret) cout << "ERROR: Camera did not capture image" << endl;
    cout << "STATUS: Camera captured image" << endl;
    ret = mySensor.GetOrientation();
    ori1 = mySensor.rotm;
    waitKey();
    ret = myCam.Capture(img2);
    if (!ret) cout << "ERROR: Camera did not capture image" << endl;
    cout << "STATUS: Camera captured image" << endl;
    ret = mySensor.GetOrientation();
    ori2 = mySensor.rotm;

    // Show and save data
    img1.Show();
    img2.Show();
    ret = img1.SavePNG("~/");
    if (!ret) cout << "ERROR: Image did not save" << endl;
    cout << "STATUS: Image saved" << endl;
    ret = img2.SavePNG("~/");
    if (!ret) cout << "ERROR: Image did not save" << endl;
    cout << "STATUS: Image saved" << endl;

    // Get keypoints and matches
    img1.FindKeypoints();
    img2.FindKeypoints();
    ret = Image::FindMatches(img1, img2, m1, m2, matches);
    Image::ShowMatches(img1, img2, matches);

    // Calculate rotation through
    // Procrustes
    ret = myRot.Estimate(m1, m2, "PROC");
    cout << myRot.rotm;
    // Gradient
    ret = myRot.Estimate(m1, m2, "GRAT");
    cout << myRot.rotm;
    // Minimization of back-projection error
    ret = myRot.Estimate(m1, m2, "MBPE");
    cout << myRot.rotm;

    // Disconnect camera and sensor
    ret = myCam.Disconnect();
    if (!ret) cout << "ERROR: Camera not disconnect" << endl;
    cout << "STATUS: Camera disconnected" << endl;
    mySensor.Disconnect();

    return 0;
}