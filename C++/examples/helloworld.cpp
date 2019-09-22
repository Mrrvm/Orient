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

    // Setup Parameters
    Mat intrinsics = (Mat_<double>(3,3) <<   1.1573e+03, -3.3579,     975.9459,
                                                        0,          1.1584e+03,  798.4888,
                                                        0,          0,           1       );
    Mat baseline = (Mat_<double>(3,1) << 0.02, -0.055, 0.055);
    Mat distcoeff = (Mat_<double>(5,1) << -0.3160, 0.1699, 3.0406e-04, 7.1815e-04, -0.0569);
    int radius = 1;
    int hcorners = 9, vcorners = 7, sqlen = 21;

    // Variables
    Sensor mySensor;
    Camera myCam;
    Image img1, img2;
    Image chessimg1, chessimg2;
    Rotation myRot(baseline, intrinsics, radius);
    vector<DMatch> matches;
    Mat Rgt1, Rgt2, rgt;
    Mat ori1, ori2;
    Mat m1, m2;
    Mat proc, mbpe, grat;
    double error;
    bool ret;

    // Connect camera 
    ret = myCam.Connect();
    if (!ret) ThrowError("Camera not connected");
    cout << YELLOW << "STATUS : " << RESET << "Camera connected" << endl;

    // Connect sensor
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;

    // Get image 1
    ret = myCam.Capture(img1);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    // Show image 1
    img1.Show();
    // Save image 1
    ret = img1.Save("img1", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    // Get chessboard image 1
    ret = myCam.Capture(chessimg1);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    chessimg1.Show();
    ret = img1.Save("chessimg1", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    // Get orientation 1
    ret = mySensor.GetOrientation();
    if (!ret) ThrowError("Sensor was not able to get orientation");
    cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
    ori1 = mySensor.eul;
    cout << BLUE << "IMU (degrees)" << RESET << ori1 << endl;

    // Get image 2
    ret = myCam.Capture(img2);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    img2.Show();
    ret = img2.Save("img2", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    // Get chessboard image 2
    ret = myCam.Capture(chessimg2);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    chessimg2.Show();
    ret = img1.Save("chessimg2", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    // Get orientation 2
    ret = mySensor.GetOrientation();
    if (!ret) ThrowError("Sensor was not able to get orientation");
    cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
    ori2 = mySensor.eul;
    cout << BLUE << "IMU (degrees)" << RESET << ori2 << endl;

    // Undistort images
    img1.Undistort(intrinsics, distcoeff);
    img2.Undistort(intrinsics, distcoeff);

    // Find keypoints in images with hessian 1000
    ret = img1.FindKeypoints(1000);
    if (!ret) ThrowError("No keypoints detected");
    cout << YELLOW << "STATUS : " << RESET << "Keypoints obtained" << endl;
    ret = img2.FindKeypoints(1000);
    if (!ret) ThrowError("No keypoints detected");
    cout << YELLOW << "STATUS : " << RESET << "Keypoints obtained" << endl;

    // Find matches between the images
    ret = Image::FindMatches(img1, img2, m1, m2, matches);
    if (!ret) ThrowError("Could not obtain matches");
    cout << YELLOW << "STATUS : " << RESET << "Matches obtained" << endl;
    // Show matches
    Image::ShowMatches(img1, img2, matches);
    // Run RANSAC to filter outliers
    ret = myRot.RansacByProcrustes(m1, m2, matches);
    Image::ShowMatches(img1, img2, matches);

    // Detect Chessboard in chessboard images
    ret = chessimg1.DetectChessboardRotation(hcorners, vcorners, sqlen, intrinsics, distcoeff, Rgt1);
    if(!ret) ThrowError("Could not detect chessboard rotation");
    cout << YELLOW << "STATUS : " << RESET << "Chessboard rotation obtained" << endl;
    ret = chessimg2.DetectChessboardRotation(hcorners, vcorners, sqlen, intrinsics, distcoeff, Rgt2);
    if(!ret) ThrowError("Could not detect chessboard rotation");
    cout << YELLOW << "STATUS : " << RESET << "Chessboard rotation obtained" << endl;
    // Find ground truth
    rgt = Rotm2Eul(Rgt2.t()*Rgt1);
    cout << BLUE << "GT (degrees)" << RESET << rgt*180/M_PI << endl;


    // Calculate rotation through
    // Procrustes
    ret = myRot.Estimate(m1, m2, "PROC");
    if (!ret) ThrowError("Could not obtain estimate through Procrustes");
    cout << YELLOW << "STATUS : " << RESET << "Procrustes estimate obtained" << endl;
    proc =  myRot.eul;
    // Gradient-Based Technique
    ret = myRot.Estimate(m1, m2, "GRAT", proc, true);
    if (!ret) ThrowError("Could not obtain estimate through Gradient-Based Technique");
    cout << YELLOW << "STATUS : " << RESET << "Gradient-Based Technique estimate obtained" << endl;
    grat = myRot.eul;
    // Minimization of back-projection error
    ret = myRot.Estimate(m1, m2, "MBPE", proc, true);
    if (!ret) ThrowError("Could not obtain estimate through MBPE");
    cout << YELLOW << "STATUS : " << RESET << "MBPE estimate obtained" << endl;
    mbpe = myRot.eul;

    // Present results
    cout << BLUE << "PROC (degrees)" << RESET << proc*180/M_PI << endl;
    cout << BLUE << "GRAT (degrees)" << RESET << grat*180/M_PI << endl;
    cout << BLUE << "MBPE (degrees)" << RESET << mbpe*180/M_PI << endl;

    error = myRot.ComputeError(rgt);
    cout << BLUE << "Error (degrees)" << RESET << error*180/M_PI << endl;

    // Disconnect camera and sensor
    ret = myCam.Disconnect();
    if (!ret) ThrowError("Camera not disconnect");
    cout << YELLOW << "STATUS :" << RESET << "Camera disconnected" << endl;
    mySensor.Disconnect();

    return 0;
}
