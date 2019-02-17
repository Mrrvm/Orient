/************
 Based on 
  https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html#feature-flann-matcher
  https://docs.opencv.org/3.0-beta/doc/user_guide/ug_features2d.html
  https://subokita.com/2014/04/07/procrustes-analysis/
 *************/

#include <fstream>
#include "defs.hpp"

#define MIN_HESSIAN 300
#define PRINT 1 

void GetCalibration(Mat& intrinsics, Mat& distCoeffs) {
    FileStorage fs("intrinsics.xml", FileStorage::READ);
    if (fs.isOpened()) {
      fs["intrinsics"] >> intrinsics;
      fs["distCoeffs"] >> distCoeffs;
      fs.release();
    }
    else {
      cout << "Running calibration ..." << endl;
      system("./get_intrinsics");
    }
}

Mat simpleProcrustes(vector<Point3f> object_points, vector<Point3f> scene_points) {
    
    // Pass to Mat format
    Mat pA = Mat(object_points);
    Mat pB = Mat(scene_points); 
   
    // Compute SVD
    Mat A = pA.reshape(1).t() * pB.reshape(1);
    Mat U, s, Vt;
    SVDecomp(A, s, U, Vt);

    Mat rotation = U * Vt;

    return rotation;
}

void convertToEuleraxis(Mat rotation) {
  float r11 = rotation.at<float>(0,0);
  float r12 = rotation.at<float>(0,1);
  float r13 = rotation.at<float>(0,2);
  float r21 = rotation.at<float>(1,0);
  float r22 = rotation.at<float>(1,1);
  float r23 = rotation.at<float>(1,2);
  float r31 = rotation.at<float>(2,0);
  float r32 = rotation.at<float>(2,1);
  float r33 = rotation.at<float>(2,2);

  float teta = acos((r11+r22+r33-1)/2);
  float e1 = (r32-r23)/(2*sin(teta));
  float e2 = (r13-r31)/(2*sin(teta));
  float e3 = (r21-r12)/(2*sin(teta));

  cout << "[" << e1 << " " << e2 << " " << e3 << " " << teta << "]" << endl;
}


int main(int argc, char *argv[]){

  // Get intrinsic parameters
  /*******************************/
  Mat intrinsics = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  GetCalibration(intrinsics, distCoeffs);
  /*******************************/

  // Undistort images and pass to grayscale
  /*******************************/
  Mat img_1;
  Mat img_2;
  if(argc == 3) {
    img_1 = imread(argv[1], IMREAD_GRAYSCALE);
    img_2 = imread(argv[2], IMREAD_GRAYSCALE);
    undistort(imread(argv[1], IMREAD_GRAYSCALE), img_1, intrinsics, distCoeffs);
    undistort(imread(argv[2], IMREAD_GRAYSCALE), img_2, intrinsics, distCoeffs); 
  }
  else {
    cout << "Usage: ./get_rotation image1 image2" << endl;
  }
  /*******************************/

  Ptr<SURF> detector = SURF::create(MIN_HESSIAN);
  Ptr<SURF> extractor = SURF::create();
  Mat descriptors_1, descriptors_2;
  vector<KeyPoint> keypoints_1, keypoints_2;
  FlannBasedMatcher matcher;
  vector<DMatch> matches, good_matches;
  int n_good_matches, last_n_good_matches = 0, true_hessian = 0;
  double max_dist = 0, min_dist = 100, dist = 0;

  // Detect keypoints using SURF
  /*******************************/
  for(int j=100; j<1000; j+=100) {

    detector = SURF::create(j);
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
    extractor->compute(img_1, keypoints_1, descriptors_1);
    extractor->compute(img_2, keypoints_2, descriptors_2);

    /*******************************/
    // Get matches using FLANN
    /*******************************/
    max_dist = 0, min_dist = 100, dist = 0;
    matcher.match(descriptors_1, descriptors_2, matches);
    // Calculation of max and min distances between keypoints
    for(int i = 0; i < descriptors_1.rows; i++) { 
      dist = matches[i].distance;
      if(dist < min_dist) min_dist = dist;
      if(dist > max_dist) max_dist = dist;
    }
    // Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    // or a small arbitary value ( 0.02 ) in the event that min_dist is very
    // small)
    for(int i = 0; i < descriptors_1.rows; i++ ) { 
      if(matches[i].distance <= max(2*min_dist, 0.02))
        n_good_matches++;
    }
    if(n_good_matches > last_n_good_matches) {
      true_hessian = j;
    }
    last_n_good_matches = n_good_matches;
  }

  #if PRINT
  cout << "Hessian is " << true_hessian << endl;
  #endif

  detector = SURF::create(true_hessian);
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);
  extractor->compute(img_1, keypoints_1, descriptors_1);
  extractor->compute(img_2, keypoints_2, descriptors_2);

  /*******************************/
  // Get matches using FLANN
  /*******************************/
  max_dist = 0, min_dist = 100, dist = 0;
  matcher.match(descriptors_1, descriptors_2, matches);
  // Calculation of max and min distances between keypoints
  for(int i = 0; i < descriptors_1.rows; i++) { 
    dist = matches[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }
  // Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  // or a small arbitary value ( 0.02 ) in the event that min_dist is very
  // small)
  for(int i = 0; i < descriptors_1.rows; i++ ) { 
    if(matches[i].distance <= max(2*min_dist, 0.02))
      good_matches.push_back(matches[i]);;
  }
  // Visualise the "good" matches
  #if PRINT
  Mat img_matches;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2,
              good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  imshow( "Good Matches", img_matches);
  for(int i = 0; i < (int)good_matches.size(); i++) { 
    printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
  }
  waitKey(0);
  #endif
  /*******************************/

  // Get keypoints from the "good" matches
  /*******************************/
  if(good_matches.size() > 5) {
      std::vector<Point3f> object_points, scene_points;
      float x, y, fx, fy, cx, cy, l, u, v;
      fx = (float)intrinsics.at<double>(0,0); 
      fy = (float)intrinsics.at<double>(1,1);
      cx = (float)intrinsics.at<double>(0,2);
      cy = (float)intrinsics.at<double>(1,2);
      
      #if PRINT
      for(u_int i = 0; i < good_matches.size(); i++) 
        cout << keypoints_1[good_matches[i].queryIdx].pt << endl << endl;
      cout << endl;
      for(u_int i = 0; i < good_matches.size(); i++) 
        cout << keypoints_2[good_matches[i].trainIdx].pt << endl << endl;
      #endif

      for(u_int i = 0; i < good_matches.size(); i++) {
        x = keypoints_1[good_matches[i].queryIdx].pt.x;
        y = keypoints_1[good_matches[i].queryIdx].pt.y;
        u = (x-cx)/fx; v = (y-cy)/fy;
        l = 1/(sqrt((u*u)+(v*v)+1));
        object_points.push_back(Point3f(u/l, v/l, 1/l));
        x = keypoints_2[good_matches[i].trainIdx].pt.x;
        y = keypoints_2[good_matches[i].trainIdx].pt.y;
        u = (x-cx)/fx; v = (y-cy)/fy;
        l = 1/(sqrt((u*u)+(v*v)+1));
        scene_points.push_back(Point3f(u/l, v/l, 1/l));
      }
      #if PRINT
      cout << object_points << endl << endl;
      cout << scene_points << endl << endl;
      #endif
      /*******************************/
      // Calculate rotation matrix using Procrustes
      /*******************************/
      Mat rotation = simpleProcrustes(object_points, scene_points);
      #if PRINT
      cout << "Rotation\n" << rotation << endl;
      #endif
      convertToEuleraxis(rotation);
      keypoints_1.clear();
      descriptors_1.release();
      descriptors_1 = descriptors_2;
      keypoints_1 = keypoints_2;
  }
  /*******************************/
  else 
    cout << "Not enough keypoints" << endl;

}

