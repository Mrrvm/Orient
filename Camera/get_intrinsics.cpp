/************
 Based on 
  http://www.aishack.in/tutorials/calibrating-undistorting-opencv-oh-yeah/
  https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
*************/

#include "defs.hpp"

#define N_SAMPLES 65
#define N_HCORNERS 7
#define N_VCORNERS 5
#define SAMPLES_DIR "chess_board/*.png"   

int main() {

    int numSquares = N_VCORNERS * N_HCORNERS;
    int i = 0, successes = 0;
    Size board_sz = Size(N_HCORNERS, N_VCORNERS);
    
    vector<vector<Point3f>> object_points;
    vector<Point3f> obj_original;
    vector<vector<Point2f>> image_points;
    vector<Point2f> corners;
    
    Mat image, gray_image;

    // Get the samples 
    cv::String path(SAMPLES_DIR);
    vector<cv::String> fn;
    vector<cv::Mat> data;
    cv::glob(path,fn,true);
    
    // Get (3D) object points (the chessboard will be the origin of the world)
    for(int j=0;j<numSquares;j++)
        obj_original.push_back(Point3f(j/N_HCORNERS, j%N_HCORNERS, 0.0f));

    // Get (2D) image points
    while(i < N_SAMPLES) {
      image = imread(fn[i]);
      cvtColor(image, gray_image, CV_BGR2GRAY);
      i++;
      // Gets corners pixel locations (the flags improve the chances of detection)
      bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
      if(found) {
          // Refine the corners
          cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
          
          /* Draw the corners for verification
          drawChessboardCorners(gray_image, board_sz, corners, found);
          imshow("win1", image);
          imshow("win2", gray_image);
          waitKey(0);
          */

          image_points.push_back(corners);
          // Push the original board points as many times as sucesses to calibrate after
          object_points.push_back(obj_original);
          successes++;
      }
    }


    if(successes > 0) {

      FileStorage fs("intrinsics.xml", FileStorage::WRITE);

      Mat intrinsics = Mat(3, 3, CV_32FC1); 
      Mat distCoeffs;
      vector<Mat> rvecs, tvecs;
      printf("Found chessboards, calibrating ...\n");
      // Camera's aspect ratio
      intrinsics.ptr<float>(0)[0] = 1;
      intrinsics.ptr<float>(1)[1] = 1;
      calibrateCamera(object_points, image_points, image.size(), intrinsics, distCoeffs, rvecs, tvecs);
      fs << "intrinsics" << intrinsics;
      fs << "distCoeffs" << distCoeffs;
      /* In case you want to print rvecs and tvecs

      myfile << "r\n";
      for (auto& vec : rvecs) {
        myfile << vec << endl;
      }
      myfile << "t\n";
      for (auto& vec : tvecs) {
        myfile << vec << endl;
      }
      */
      fs.release();
    }
    else 
      cout << "No successes." << endl;
}