#include "defs.hpp"
#include <ueye.h>

#define N_SAMPLES 70
#define SAMPLES_DIR "calibrate_img/"

#define HEIGHT 1542
#define WIDTH 2056
#define BITSPIXEL 32

void spawn_error(int camHandle, std::string where) {
	char *ppcErr;
	int pErr = 0;
	is_GetError(camHandle, &pErr, &ppcErr);
 	std::cout << "Error at " << where << ppcErr << std::endl;
 	exit(EXIT_FAILURE);
}

void get_images(int n_images) {

	char *img = NULL;
	int pid = 0;
	Mat imgMat(HEIGHT, WIDTH, CV_8UC4);

	// Setting 0 allows the first available camera to be initialized or selected
	// Otherwise it represents the camera ID you want to initialize
	HIDS camHandle= (HIDS)0;
	int nRet = is_InitCamera(&camHandle, NULL);

	if (nRet != IS_SUCCESS) {
 	//Check if GigE uEye SE needs a new starter firmware
		if (nRet == IS_STARTER_FW_UPLOAD_NEEDED) {
			// Calculate time needed for updating the starter firmware for display 
			int nTime = 0;
			is_GetDuration (camHandle, IS_SE_STARTER_FW_UPLOAD, &nTime);
			std::cout << "Updating for " << nTime << std::endl;
		   	// Upload new starter firmware during initialization
		   	camHandle = camHandle | IS_ALLOW_STARTER_FW_UPLOAD;
			nRet = is_InitCamera (&camHandle, NULL);
	 	}
	 	else 
	 		spawn_error(camHandle, "is_InitCamera");
	}
	else
		std::cout << "Camera is connected" << std::endl;


	// Allocate memory for image
	if(is_AllocImageMem(camHandle, WIDTH, HEIGHT, BITSPIXEL, &img, &pid) != IS_SUCCESS) 
		spawn_error(camHandle, "is_AllocImageMem");
	// Set image memory 
	if(is_SetImageMem(camHandle, img, pid) != IS_SUCCESS) 
		spawn_error(camHandle, "is_SetImageMem");
	// Capture image 
	if(is_FreezeVideo(camHandle, IS_WAIT) != IS_SUCCESS)
		spawn_error(camHandle, "is_FreezeVideo");

	// Convert the image in memory to an OpenCV Mat variable
	int j = 0, i = 0, m = 0;
	while(j < HEIGHT) {
		while(i<WIDTH*4)
		{
			if(i%4 != 3) {
				imgMat.ptr(j)[i] = img[j*WIDTH*4+m];
				
				++m;
			}
			++i;
		}
		i=0;
		j++;
		m=0;
	}

	// Visualise the data
	//namedWindow("Image", 1);
    //imshow("Image", imgMat);
	//waitKey();

	imwrite("calibrate_img/img.jpeg", imgMat);

	// Free memory
	if(is_FreeImageMem(camHandle, img, pid) != IS_SUCCESS)
		spawn_error(camHandle, "is_FreeImageMem");

	is_ExitCamera(camHandle);
	return;
}

int main(int argc, char const *argv[]) {
	
	get_images(N_SAMPLES);





	return 0;
}