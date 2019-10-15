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

    // Setup Parameters
    Mat intrinsics = (Mat_<double>(3,3) <<   1.1253e+03, 0.945541684501329,    9.960929796822086e+02,
            0,           1.125630873153923e+03, 7.543243830849593e+02,
            0,           0,                     1                  );
    Mat baseline = (Mat_<double>(3,1) << 0.02, -0.055, 0.055);
    Mat distcoeff = (Mat_<double>(5,1) << -0.3007, 0.1288, 4.7003e-04, -3.3083e-04, -0.0318);
    int radius = 3;
    int hcorners = 10, vcorners = 7, sqlen = 20;

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
    double eproc, embpe, egrat;
    bool ret;

#if online
    // Connect camera and sensor
    ret = myCam.Connect();
    if (!ret) ThrowError("Camera not connected");
    cout << YELLOW << "STATUS : " << RESET << "Camera connected" << endl;

    getchar();

    // Get image 1
    ret = myCam.Capture(img1);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    img1.Show();
    ret = img1.Save("img1", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    getchar();

    // Get chessboard image 1
    ret = myCam.Capture(chessimg1);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    chessimg1.Show();
    ret = chessimg1.Save("chessimg1", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    getchar();

    // Get image 2
    ret = myCam.Capture(img2);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    img2.Show();
    ret = img2.Save("img2", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    getchar();

    // Get chessboard image 2
    ret = myCam.Capture(chessimg2);
    if (!ret) ThrowError("Camera did not capture image");
    cout << YELLOW << "STATUS : " << RESET << "Camera captured image" << endl;
    chessimg2.Show();
    ret = chessimg2.Save("chessimg2", "/home/imarcher/", "jpg");
    if (!ret) ThrowError("Image did not save");
    cout << YELLOW << "STATUS : " << RESET << "Image saved" << endl;

    getchar();
#endif

#if not online
    img1.Load("image1", "/home/imarcher/", "png");
    img2.Load("image2", "/home/imarcher/", "png");
    chessimg1.Load("image1", "/home/imarcher/", "png");
    chessimg2.Load("image2", "/home/imarcher/", "png");
#endif

    // Get keypoints and matches
    img1.Undistort(intrinsics, distcoeff);
    img2.Undistort(intrinsics, distcoeff);
    ret = img1.FindKeypoints();
    if (!ret) ThrowError("No keypoints detected");
    cout << YELLOW << "STATUS : " << RESET << "Keypoints obtained" << endl;
    ret = img2.FindKeypoints();
    if (!ret) ThrowError("No keypoints detected");
    cout << YELLOW << "STATUS : " << RESET << "Keypoints obtained" << endl;
    ret = Image::FindMatches(img1, img2, m1, m2, matches);
    if (!ret) ThrowError("Could not obtain matches");
    cout << YELLOW << "STATUS : " << RESET << "Matches obtained" << endl;
    //Image::ShowMatches(img1, img2, matches);

    // Run RANSAC to filter outliers
    ret = myRot.RansacByProcrustes(m1, m2, matches);
    if (!ret) ThrowError("Could not filter matches");
    cout << YELLOW << "STATUS : " << RESET << "Matches filtered" << endl;
    Image::ShowMatches(img1, img2, matches);

    // Determine ground truth
    ret = chessimg1.DetectChessboardRotation(hcorners, vcorners, sqlen, intrinsics, distcoeff, Rgt1);
    if(!ret) ThrowError("Could not detect chessboard rotation");
    cout << YELLOW << "STATUS : " << RESET << "Chessboard rotation obtained" << endl;
    ret = chessimg2.DetectChessboardRotation(hcorners, vcorners, sqlen, intrinsics, distcoeff, Rgt2);
    if(!ret) ThrowError("Could not detect chessboard rotation");
    cout << YELLOW << "STATUS : " << RESET << "Chessboard rotation obtained" << endl;
    rgt = Rotm2Eul(Rgt1.t()*Rgt2);
    cout << rgt*180/M_PI << endl;

    // Calculate rotation through
    // Procrustes
    ret = myRot.Estimate(m1, m2, "PROC");
    if (!ret) ThrowError("Could not obtain estimate through Procrustes");
    cout << YELLOW << "STATUS : " << RESET << "Procrustes estimate obtained" << endl;
    myRot.eul.copyTo(proc);
    eproc = myRot.ComputeError(rgt);
    cout << proc*180/M_PI << endl;
    exit(0);

    // Gradient-Based Technique
    ret = myRot.Estimate(m1, m2, "GRAT", proc, false);
    if (!ret) ThrowError("Could not obtain estimate through Gradient-Based Technique");
    cout << YELLOW << "STATUS : " << RESET << "Gradient-Based Technique estimate obtained" << endl;
    myRot.eul.copyTo(grat);
    egrat = myRot.ComputeError(rgt);

    // Minimization of back-projection error
    ret = myRot.Estimate(m1, m2, "MBPE", proc, false);
    if (!ret) ThrowError("Could not obtain estimate through MBPE");
    cout << YELLOW << "STATUS : " << RESET << "MBPE estimate obtained" << endl;
    myRot.eul.copyTo(mbpe);
    embpe = myRot.ComputeError(rgt);

    cout << BLUE << "Rotation GT \t(deg)\t"   << RESET << rgt*180/M_PI << endl;
    cout << BLUE << "Rotation PROC \t(deg)\t" << RESET << proc*180/M_PI << endl;
    cout << BLUE << "Rotation GRAT \t(deg)\t" << RESET << grat*180/M_PI << endl;
    cout << BLUE << "Rotation MBPE \t(deg)\t" << RESET << mbpe*180/M_PI << endl;
    cout << BLUE << "Error PROC \t(deg)\t" << RESET << eproc*180/M_PI << endl;
    cout << BLUE << "Error GRAT \t(deg)\t" << RESET << egrat*180/M_PI << endl;
    cout << BLUE << "Error MBPE \t(deg)\t" << RESET << embpe*180/M_PI << endl;

#if online
    // Disconnect camera and sensor
    ret = myCam.Disconnect();
    if (!ret) ThrowError("Camera not disconnect");
    cout << YELLOW << "STATUS :" << RESET << "Camera disconnected" << endl;
#endif
    return 0;
}
