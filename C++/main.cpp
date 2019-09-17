#include "camera.h"
#include "image.h"
#include "sensor.h"
#include "rotation.h"
#include "defs.h"

void ThrowError(string what) {
    cout << RED << "(main.cpp) ERROR : " << RESET << what << endl;
    exit(EXIT_FAILURE);
}

int main() {

    Sensor mySensor;
    Camera myCam;
    Image img1("img1", "/home/imarcher/", "jpg", 1000), img2("img2", "/home/imarcher/", "jpg", 1000);
    Mat ori1, ori2;
    Mat m1, m2;
    vector<DMatch> matches;
    Mat intrinsics = (Mat_<double>(3,3) <<   1.1573e+03, -3.3579,     975.9459,
                                                        0,          1.1584e+03,  798.4888,
                                                        0,          0,           1       );
    Mat baseline = (Mat_<double>(3,1) << 0.02, -0.055, 0.055);
    Mat tanDist = (Mat_<double>(3,1) << 3.0406e-04, 7.1815e-04);
    Mat radDist = (Mat_<double>(3,1) << -0.3160, 0.1699, -0.0569);
    int radius = 1;
    Rotation myRot(baseline, intrinsics, radius);
    bool ret;
    Mat proc, mbpe, grat;
/*
    // Connect camera and sensor
    ret = myCam.Connect();
    if (!ret) ThrowError("Camera not connected");
    cout << YELLOW << "STATUS : " << RESET << "Camera connected" << endl;

    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;


    // Get image 1
    ret = myCam.Capture(img1);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    img1.Show();
    waitKey(0);
    ret = img1.Save("/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    // Get orientation 1
    ret = mySensor.GetOrientation();
    ori1 = mySensor.rotm;
    cout << ori1 << endl;

    // Get image 2
    ret = myCam.Capture(img2);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    img2.Show();
    waitKey(0);
    ret = img2.Save("/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    // Get orientation 2
    ret = mySensor.GetOrientation();
    ori2 = mySensor.rotm;
    cout << ori2 << endl
*/

    // Read image 1 & 2
    img1.Load();
    img2.Load();

    // Get keypoints and matches
    img1.FindKeypoints();
    img2.FindKeypoints();
    ret = Image::FindMatches(img1, img2, m1, m2, matches);
    if (!ret) ThrowError("Could not obtain matches");
    cout << YELLOW << "STATUS : " << RESET << "Matches obtained" << endl;
    //Image::ShowMatches(img1, img2, matches);
    //waitKey(0);

    cout << m1 << endl;

    // Calculate rotation through:
    // Procrustes
    ret = myRot.Estimate(m1, m2, "PROC");
    if (!ret) ThrowError("Could not obtain estimate through Procrustes");
    cout << YELLOW << "STATUS : " << RESET << "Procrustes estimate obtained" << endl;
    proc =  myRot.eul;

    // Gradient-Based Technique
    ret = myRot.Estimate(m1, m2, "GRAT", proc);
    if (!ret) ThrowError("Could not obtain estimate through Gradient-Based Technique");
    cout << YELLOW << "STATUS : " << RESET << "Gradient-Based Technique estimate obtained" << endl;
    grat = myRot.eul;

    // Minimization of back-projection error
    Mat M1 = myRot.ProjectToSphere(m1);
    ret = myRot.Estimate(m1, m2, "MBPE", proc, M1.row(2));
    if (!ret) ThrowError("Could not obtain estimate through MBPE");
    cout << YELLOW << "STATUS : " << RESET << "MBPE estimate obtained" << endl;
    mbpe = myRot.eul;

    cout << BLUE << "PROC (degrees)" << RESET << proc*180/M_PI << endl;
    cout << BLUE << "GRAT (degrees)" << RESET << grat*180/M_PI << endl;
    cout << BLUE << "MBPE (degrees)" << RESET << mbpe*180/M_PI << endl;

    // Disconnect camera and sensor
    //ret = myCam.Disconnect();
    //if (!ret) ThrowError("Camera not disconnect");
    //cout << YELLOW << "STATUS :" << RESET << "Camera disconnected" << endl;
    //mySensor.Disconnect();

    return 0;
}