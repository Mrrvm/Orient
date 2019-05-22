#include "defs.hpp"
#include <ueye.h>

#define HEIGHT 1542
#define WIDTH 2056
#define BITSPIXEL 32
#define MIN_HESSIAN 300

void spawn_error(int cameraHandle, std::string where) {
	char *ppcErr;
	int pErr = 0;
	is_GetError(cameraHandle, &pErr, &ppcErr);
 	std::cout << "Error at " << where << ppcErr << std::endl;
 	//exit(EXIT_FAILURE);
}

Mat capture_image(HIDS &cameraHandle) {

	char *img = NULL;
	int pid = 0;
	Mat matImg(HEIGHT, WIDTH, CV_8UC4);

	// Allocate memory for  image
	if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &img, &pid) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_AllocImageMem");
	// Set image memory for Previous image
	if(is_SetImageMem(cameraHandle, img, pid) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_SetImageMem");
	// Capture image for Previous image
	if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreezeVideo");

	// Convert the image in memory to an OpenCV Mat variable
	int j = 0, i = 0, m = 0;
	while(j < HEIGHT) {
		while(i<WIDTH*4)
		{
			if(i%4 != 3) {
				matImg.ptr(j)[i] = img[j*WIDTH*4+m];
				++m;
			}
			else {
				matImg.ptr(j)[i] = img[j*WIDTH*4+i+WIDTH*3];
			}
			++i;
		}
		i=0;
		j++;
		m=0;
	}

	// Free memory
	if(is_FreeImageMem(cameraHandle, img, pid) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreeImageMem");

	return matImg;
}

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

int find_best_hessian(Mat img_1, Mat img_2) {

	Mat descriptors_1, descriptors_2;
	vector<KeyPoint> keypoints_1, keypoints_2;
	FlannBasedMatcher matcher;
	vector<DMatch> matches, good_matches;
	int n_good_matches, last_n_good_matches = 0, true_hessian = 0;
	double max_dist = 0, min_dist = 100, dist = 0;
	Ptr<SURF> detector, extractor;

	for(int j=100; j<1000; j+=100) {

		// Get keypoints using SURF
		detector = SURF::create(j);
		extractor = SURF::create();
		detector->detect(img_1, keypoints_1);
		detector->detect(img_2, keypoints_2);
		extractor->compute(img_1, keypoints_1, descriptors_1);
		extractor->compute(img_2, keypoints_2, descriptors_2);

		// Get matches using FLANN
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
	return true_hessian;
}


Mat compute_rotation(Mat matImg1, Mat matImg2) {
	
	Mat rotation;
	Mat grey_img1, img_1;
  	Mat grey_img2, img_2;

  	// Get intrinsic parameters
	Mat intrinsics = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	GetCalibration(intrinsics, distCoeffs);

	cv::cvtColor(matImg1, grey_img1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(matImg2, grey_img2, cv::COLOR_BGR2GRAY);
	undistort(grey_img1, img_1, intrinsics, distCoeffs);
    undistort(grey_img2, img_2, intrinsics, distCoeffs); 

	Mat descriptors_1, descriptors_2;
	vector<KeyPoint> keypoints_1, keypoints_2;
	FlannBasedMatcher matcher;
	vector<DMatch> matches, good_matches;
	int n_good_matches, last_n_good_matches = 0, true_hessian = 800;
	double max_dist = 0, min_dist = 100, dist = 0;

	// TODO: erase if it takes too long
	//true_hessian = find_best_hessian(matImg1, matImg2);

	// Get all keypoints using SURF
	Ptr<SURF> detector = SURF::create(true_hessian);
	Ptr<SURF> extractor = SURF::create();
  	detector->detect(img_1, keypoints_1);
  	detector->detect(img_2, keypoints_2);
  	extractor->compute(img_1, keypoints_1, descriptors_1);
  	extractor->compute(img_2, keypoints_2, descriptors_2);

  	// Get matches using FLANN
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

	if(good_matches.size() > 5) {
		std::vector<Point3f> object_points, scene_points;
		float x, y, fx, fy, cx, cy, l, u, v;
		fx = (float)intrinsics.at<double>(0,0); 
		fy = (float)intrinsics.at<double>(1,1);
		cx = (float)intrinsics.at<double>(0,2);
		cy = (float)intrinsics.at<double>(1,2);

		// Sphere projection
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

		// Calculate rotation matrix using Procrustes
		rotation = simpleProcrustes(object_points, scene_points);
		
  	}

  	return rotation;
}

int main(int argc, char const *argv[]) {
	
	Mat matImg1(HEIGHT, WIDTH, CV_8UC4);
	Mat matImg2(HEIGHT, WIDTH, CV_8UC4);
	Mat rotation;

	// Initialize camera
	HIDS cameraHandle = (HIDS)0;
	int nRet = is_InitCamera(&cameraHandle, NULL);
	if (nRet != IS_SUCCESS) {
 	//Check if GigE uEye SE needs a new starter firmware
		if (nRet == IS_STARTER_FW_UPLOAD_NEEDED) {
			// Calculate time needed for updating the starter firmware for display 
			int nTime = 0;
			is_GetDuration (cameraHandle, IS_SE_STARTER_FW_UPLOAD, &nTime);
			std::cout << "Updating for " << nTime << std::endl;
		   	// Upload new starter firmware during initialization
		   	cameraHandle = cameraHandle | IS_ALLOW_STARTER_FW_UPLOAD;
			nRet = is_InitCamera (&cameraHandle, NULL);
	 	}
	 	else 
	 		spawn_error(cameraHandle, "is_InitCamera");
	}
	else
		std::cout << "Camera is connected" << std::endl;

	// Capture first image of all
	matImg1 = capture_image(cameraHandle);
	
	while(1) {

		// TODO: Define when to take the second image
		matImg2 = capture_image(cameraHandle);

		rotation = compute_rotation(matImg1, matImg2);

		matImg1.release();
		matImg1 = matImg2;
		matImg2.release();

		cout << rotation << endl;		
	}

	return 0;
}