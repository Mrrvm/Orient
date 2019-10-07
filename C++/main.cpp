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
    Mat intrinsics = (Mat_<double>(3,3) <<
            1.1253e+03,  0.945541684501329,     9.960929796822086e+02,
            0,           1.125630873153923e+03, 7.543243830849593e+02,
            0,           0,                     1                  );
    Mat baseline = (Mat_<double>(3,1) << 0.02, -0.055, 0.055);
    Mat distcoeff = (Mat_<double>(5,1) << -0.3007, 0.1288, 4.7003e-04, -3.3083e-04, -0.0318);
    int radius = 4;
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
    img1.Load("0d", "/home/imarcher/", "png");
    img2.Load("0d", "/home/imarcher/", "png");
    chessimg1.Load("0d", "/home/imarcher/", "png");
    chessimg2.Load("y15.39+", "/home/imarcher/", "png");
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
    //Image::ShowMatches(img1, img2, matches);

    // Run RANSAC to filter outliers
    ret = myRot.RansacByProcrustes(m1, m2, matches);
    if (!ret) ThrowError("Could not filter matches");
    cout << YELLOW << "STATUS : " << RESET << "Matches filtered" << endl;
    Image::ShowMatches(img1, img2, matches);

    exit(0);

    // Determine ground truth
    ret = chessimg1.DetectChessboardRotation(hcorners, vcorners, sqlen, intrinsics, distcoeff, Rgt1);
    if(!ret) ThrowError("Could not detect chessboard rotation");
    cout << YELLOW << "STATUS : " << RESET << "Chessboard rotation obtained" << endl;
    ret = chessimg2.DetectChessboardRotation(hcorners, vcorners, sqlen, intrinsics, distcoeff, Rgt2);
    if(!ret) ThrowError("Could not detect chessboard rotation");
    cout << YELLOW << "STATUS : " << RESET << "Chessboard rotation obtained" << endl;
    rgt = Rotm2Eul(Rgt2.t()*Rgt1);
/*
    m1 = (Mat_<double>(2,20) <<
            1046.29321289063,	1102.26586914063,	960.314819335938,	383.154357910156,	1099.29211425781,	311.241546630859,	378.382110595703,	1091.93542480469,	1061.01184082031,	953.992065429688,	1049.57299804688,	261.617523193359,	1062.81689453125,	959.232177734375,	352.559509277344,	439.344665527344,	1068.60351562500,	982.531860351563,	477.119873046875,	978.980712890625,
    361.145477294922,	359.245239257813,	367.472778320313,	827.739440917969,	335.323089599609,	814.914916992188,	892.750122070313,	347.243927001953,	350.147430419922,	367.128204345703,	368.170532226563,	793.261718750000,	360.690979003906,	355.510559082031,	820.996704101563,	392.840728759766,	330.582916259766,	362.757812500000,	428.392761230469,	357.300781250000);

    m2 = (Mat_<double>(2,20) << 742.006042480469,	784.128234863281,	679.208740234375,	471.873077392578,	769.063598632813,	393.572814941406,	503.241363525391,	767.938354492188,	748.249145507813,	672.610229492188,	748.695739746094,	337.977447509766,	754.532470703125,	671.380432128906,	437.944061279297,	281.680664062500,	741.120239257813,	694.895751953125,	331.852020263672,	687.131103515625,
    591.555480957031,	555.007202148438,	648.333374023438,	1410.75610351563,	539.189941406250,	1454.13488769531,	1475.87463378906,	552.944580078125,	573.004089355469,	652.720703125000,	594.308227539063,	1471.95739746094,	581.857849121094,	642.833312988281,	1427.01586914063,	991.724975585938,	555.171752929688,	632.227600097656,	997.700805664063,	631.369140625000);
*/
    // Calculate rotation through
    // Procrustes
    ret = myRot.Estimate(m1, m2, "PROC");
    if (!ret) ThrowError("Could not obtain estimate through Procrustes");
    cout << YELLOW << "STATUS : " << RESET << "Procrustes estimate obtained" << endl;
    myRot.eul.copyTo(proc);
    eproc = myRot.ComputeError(rgt);

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
