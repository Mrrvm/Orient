#include <ueye.h>
#include <thread>
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#include "Camera/defs.hpp"

#define HEIGHT 480
#define WIDTH 752
#define BITSPIXEL 32
#define MIN_HESSIAN 400

vector<KeyPoint> prev_keypoints;
Mat prev_descriptors;
Mat intrinsics = Mat(3, 3, CV_32FC1);
Mat distCoeffs;

void spawn_error(int cameraHandle, std::string where) {
	char *ppcErr;
	int pErr = 0;
	is_GetError(cameraHandle, &pErr, &ppcErr);
 	cout << "Error at " << where << ppcErr << std::endl;
 	exit(EXIT_FAILURE);
}

Mat get_image() {

	char *img = NULL;
	int pid = 0;
	Mat i_mat(HEIGHT, WIDTH, CV_8UC4);
	HIDS cameraHandle= (HIDS)0;
	int nRet = is_InitCamera(&cameraHandle, NULL);

	if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &img, &pid) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_AllocImageMem");
	
	if(is_SetImageMem(cameraHandle, img, pid) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_SetImageMem");

	if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreezeVideo");
	else
		std::cout << "Image was captured!" << std::endl;

	// Convert the image in memory to an OpenCV Mat variable
	int j = 0, i = 0, m = 0;
	while(j < HEIGHT) {
		while(i<WIDTH*4) {
			if(i%4 != 3) {
				i_mat.ptr(j)[i] = img[j*WIDTH*4+m];
				++m;
			}
			else {
				i_mat.ptr(j)[i] = img[j*WIDTH*4+i+WIDTH*3];
			}
			++i;
		}
		i=0;j++;m=0;
	}

	// Free memory
	if(is_FreeImageMem(cameraHandle, img, pid) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreeImageMem");

	is_ExitCamera(cameraHandle);
	
	return i_mat;
}

void GetCalibration(Mat& intrinsics, Mat& distCoeffs) {
    FileStorage fs("../Camera/intrinsics.xml", FileStorage::READ);
    if (fs.isOpened()) {
      fs["intrinsics"] >> intrinsics;
      fs["distCoeffs"] >> distCoeffs;
      fs.release();
    }
    else {
      cout << "Running calibration ..." << endl;
      system("./../Camera/get_intrinsics");
      GetCalibration(intrinsics, distCoeffs);
    }
    return;
}

Mat simpleProcrustes(vector<Point3f> object_points, vector<Point3f> scene_points) {
    
    // Pass to Mat format
    Mat pA = Mat(object_points);
    Mat pB = Mat(scene_points); 
   
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

void get_kpts(Mat& descriptors, vector<KeyPoint>& kpts) {

	Mat im(HEIGHT, WIDTH, CV_8UC4);
	Mat im_undis;
    im = get_image(); 

  	GetCalibration(intrinsics, distCoeffs);
  	cvtColor(im, im, COLOR_BGR2GRAY);
    undistort(im, im_undis, intrinsics, distCoeffs);


  	Ptr<SURF> detector = SURF::create(MIN_HESSIAN);
  	Ptr<SURF> extractor = SURF::create();
	detector->detect(im_undis, kpts);
	extractor->compute(im_undis, kpts, descriptors);

	return;
}

void camera_rotation(Mat& rotation) {

	Mat descriptors;
  	vector<KeyPoint> keypoints;
  	get_kpts(descriptors, keypoints);
  	FlannBasedMatcher matcher;
  	vector<DMatch> matches, good_matches;
  	double max_dist = 0, min_dist = 100, dist = 0;
  	matcher.match(prev_descriptors, descriptors, matches);
  	if(matches.size() > 5) {
		for(int i = 0; i < descriptors.rows; i++) { 
			dist = matches[i].distance;
			if(dist < min_dist) min_dist = dist;
			if(dist > max_dist) max_dist = dist;
		}
		for(int i = 0; i < descriptors.rows; i++ ) { 
			if(matches[i].distance <= max(2*min_dist, 0.02))
			  good_matches.push_back(matches[i]);
		}
		if(good_matches.size() > 5) {
		  	std::vector<Point3f> object_points, scene_points;
		  	  float x, y, fx, fy, cx, cy, l, u, v;
			  fx = (float)intrinsics.at<double>(0,0); 
			  fy = (float)intrinsics.at<double>(1,1);
			  cx = (float)intrinsics.at<double>(0,2);
			  cy = (float)intrinsics.at<double>(1,2);
			for(u_int i = 0; i < good_matches.size(); i++ ) {
				x = prev_keypoints[good_matches[i].queryIdx].pt.x;
				y = prev_keypoints[good_matches[i].queryIdx].pt.y;
		      	u = (x-cx)/fx; v = (y-cy)/fy;
		      	l = 1/(sqrt((u*u)+(v*v)+1));
		      	object_points.push_back(Point3f(u/l, v/l, 1/l));
				x = keypoints[good_matches[i].trainIdx].pt.x;
				y = keypoints[good_matches[i].trainIdx].pt.y;
		        u = (x-cx)/fx; v = (y-cy)/fy;
		        l = 1/(sqrt((u*u)+(v*v)+1));
		        scene_points.push_back(Point3f(u/l, v/l, 1/l));
			}
		  	rotation = simpleProcrustes(object_points, scene_points);
		  	prev_keypoints.clear();
		  	prev_descriptors.release();
		  	prev_descriptors = descriptors;
		  	prev_keypoints = keypoints;
		}
  		return;
  	}
  	cout << "Not enough keypoints" << endl;
  	return;
}


int main(int argc, char *argv[]) {

	ImuData sdata, prev_sdata;
	Mat rotation;
	// Initialize
  	get_kpts(prev_descriptors, prev_keypoints);
	//LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
	//LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_U, "A1019SCB");
	//lpms->setConfigurationPrm(41, 3);
	cout << "Initialized" << endl;

	while(1) {
		getchar();
		camera_rotation(rotation);
		cout << "Camera rotation" << endl;
		cout << rotation << endl;
/*
		if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
				lpms->hasImuData()) {		
			    thread t1(camera_rotation, ref(rotation));
				sdata = lpms->getCurrentData();
			    t1.join();
			    cout << "Sensor rotation" << endl;
			    printf("[%f %f %f;\n%f %f %f;\n%f %f %f]\n", 
					sdata.rotationM[0], sdata.rotationM[1], sdata.rotationM[2],
					sdata.rotationM[3], sdata.rotationM[4], sdata.rotationM[5], 
					sdata.rotationM[6], sdata.rotationM[7], sdata.rotationM[8]);
			    cout << "Camera rotation" << endl;
			    cout << rotation << endl;
			    prev_sdata = sdata;
		}
		else {
			cout << "Sensor connection refused." << endl;
		}*/
	}

	//manager->removeSensor(lpms);
	//delete manager;
	return 0;
}