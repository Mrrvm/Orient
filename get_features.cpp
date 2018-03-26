// https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html#feature-flann-matcher
// https://docs.opencv.org/3.0-beta/doc/user_guide/ug_features2d.html

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"  

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main( int argc, char** argv ) {

  Mat img_1 = imread(argv[1], IMREAD_GRAYSCALE);
  Mat img_2 = imread(argv[2], IMREAD_GRAYSCALE);
  Mat descriptors_1, descriptors_2;
  Mat img_matches;
  int minHessian = 1000;
  double max_dist = 0, min_dist = 100, dist = 0;
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  std::vector<DMatch> matches, good_matches;
  Ptr<SURF> extractor = SURF::create();

  // Detect the keypoints using SURF Detector
  Ptr<SURF> detector = SURF::create(minHessian);
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  // Calculate descriptors (feature vectors)
  extractor->compute(img_1, keypoints_1, descriptors_1);
  extractor->compute(img_2, keypoints_2, descriptors_2);

  // Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  matcher.match(descriptors_1, descriptors_2, matches);

  // Quick calculation of max and min distances between keypoints
  for(int i = 0; i < descriptors_1.rows; i++) { 
    dist = matches[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  for(int i = 0; i < descriptors_1.rows; i++ ) { 
    if(matches[i].distance <= max(2*min_dist, 0.02))
      good_matches.push_back( matches[i]);
  }

  // Draw only "good" matches
  drawMatches(img_1, keypoints_1, img_2, keypoints_2,
              good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  // Show detected matches
  imshow( "Good Matches", img_matches);

  for(int i = 0; i < (int)good_matches.size(); i++) { 
    printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
  }

  waitKey(0);

/*
  // Draw keypoints
  Mat img_keypoints_1; 
  Mat img_keypoints_2;
  drawKeypoints(img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  drawKeypoints(img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

  // Show keypoints
  imshow("Keypoints 1", img_keypoints_1 );
  imshow("Keypoints 2", img_keypoints_2 );

  waitKey(0);
*/
  return 0;
}

