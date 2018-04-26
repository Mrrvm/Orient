/************
 Based on 
  https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html#feature-flann-matcher
  https://docs.opencv.org/3.0-beta/doc/user_guide/ug_features2d.html
  https://subokita.com/2014/04/07/procrustes-analysis/
 *************/

#include <fstream>
#include "defs.hpp"

#define MIN_HESSIAN 800

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

Mat simpleProcrustes(vector<Point2f> object_points, vector<Point2f> scene_points) {
    
    // Pass to Mat format
    Mat pA = Mat(object_points).reshape(1);
    Mat pB = Mat(scene_points).reshape(1); 

    // Turn into homogenous coordinates
    
    Mat ones = Mat(pA.rows, 1, pA.type(), 1);
    hconcat(pA, ones, pA);
    hconcat(pB, ones, pB);
    cout << pA << endl; 
    cout << pB << endl; 
    
    // Recenter the points based on their mean 
    Scalar mu_A = mean(pA);  
    Mat pA0 = pA - Mat(pA.size(), pA.type(), mu_A);
    Scalar mu_B = mean(pB);
    Mat pB0 = pB - Mat(pB.size(), pB.type(), mu_B);

    // Normalize them 
    float norm_pA = norm(pA0);
    pA0 /= norm_pA;
    float norm_pB = norm(pB0);
    pB0 /= norm_pB;

    // Compute SVD
    Mat A = pA0.reshape(1).t() * pB0.reshape(1);
    Mat U, s, Vt;
    SVDecomp(A, s, U, Vt);
    Mat rotation = Vt.t() * U.t();

    return rotation;
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
    undistort(imread(argv[1], IMREAD_GRAYSCALE), img_1, intrinsics, distCoeffs);
    undistort(imread(argv[2], IMREAD_GRAYSCALE), img_2, intrinsics, distCoeffs); 
  }
  else {
    cout << "Usage: ./get_rotation image1 image2" << endl;
  }
  /*******************************/

  // Detect keypoints using SURF
  /*******************************/
  Ptr<SURF> detector = SURF::create(MIN_HESSIAN);
  Ptr<SURF> extractor = SURF::create();
  Mat descriptors_1, descriptors_2;
  vector<KeyPoint> keypoints_1, keypoints_2;

  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);
  extractor->compute(img_1, keypoints_1, descriptors_1);
  extractor->compute(img_2, keypoints_2, descriptors_2);
  /*******************************/

  // Get matches using FLANN
  /*******************************/
  FlannBasedMatcher matcher;
  vector<DMatch> matches, good_matches;
  double max_dist = 0, min_dist = 100, dist = 0;

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
      good_matches.push_back(matches[i]);
  }
  // Visualise the "good" matches
  Mat img_matches;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2,
              good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  imshow( "Good Matches", img_matches);
  for(int i = 0; i < (int)good_matches.size(); i++) { 
    printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
  }
  waitKey(0);
  /*******************************/

  // Get keypoints from the "good" matches
  /*******************************/
  std::vector<Point2f> object_points, scene_points;
  float l1, l2, x, y;
  for(u_int i = 0; i < good_matches.size(); i++ ) {
      x = keypoints_1[good_matches[i].queryIdx].pt.x;
      y = keypoints_1[good_matches[i].queryIdx].pt.y;
      l1 = 1/(sqrt((x*x)+(y*y)));
      x = keypoints_2[good_matches[i].trainIdx].pt.x;
      y = keypoints_2[good_matches[i].trainIdx].pt.y;
      l2 = 1/(sqrt((x*x)+(y*y)));
      object_points.push_back(keypoints_1[good_matches[i].queryIdx].pt);
      scene_points.push_back((l2/l1)*keypoints_2[good_matches[i].trainIdx].pt);
  }
  /*******************************/

  // Calculate rotation matrix using Procrustes
  /*******************************/
  Mat rotation = simpleProcrustes(object_points, scene_points);
  cout << "Rotation\n" << rotation << endl;
  /*******************************/

}

