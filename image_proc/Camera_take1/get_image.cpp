#include "defs.hpp"
#include <ueye.h>

#define HEIGHT 1542
#define WIDTH 2056
#define BITSPIXEL 32

void spawn_error(int cameraHandle, std::string where) {
	char *ppcErr;
	int pErr = 0;
	is_GetError(cameraHandle, &pErr, &ppcErr);
 	std::cout << "Error at " << where << ppcErr << std::endl;
 	//exit(EXIT_FAILURE);
}

int main( int argc, char** argv ) {

	char *ppcImgActive = NULL, *ppcImgPrevious = NULL;
	int pidActive = 0, pidPrevious = 0;
	Mat matPrevious(HEIGHT, WIDTH, CV_8UC4);
	Mat matActive(HEIGHT, WIDTH, CV_8UC4);
	// Setting 0 allows the first available camera to be initialized or selected
	// Otherwise it represents the camera ID you want to initialize
	HIDS cameraHandle= (HIDS)0;
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

	// Allocate memory for active and previous image
	if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &ppcImgActive, &pidActive) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_AllocImageMem");
	if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &ppcImgPrevious, &pidPrevious) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_AllocImageMem");

	// Set image memory for Previous image
	if(is_SetImageMem(cameraHandle, ppcImgPrevious, pidPrevious) != IS_SUCCESS) 
		spawn_error(cameraHandle, "is_SetImageMem");
	// Capture image for Previous image
	if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreezeVideo");
	else
		std::cout << "Previous Image was captured!" << std::endl;

	// Here there will be a time difference (maybe use captureVideo instead?)
	// What will the frame rate be?

	// Set image memory for Active image
	if(is_SetImageMem(cameraHandle, ppcImgActive, pidActive) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_SetImageMem");
	// Capture image for Active image
	if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreezeVideo");
	else
		std::cout << "Active Image was captured!" << std::endl;


	// Convert the image in memory to an OpenCV Mat variable
	int j = 0, i = 0, m = 0;
	while(j < HEIGHT) {
		while(i<WIDTH*4)
		{
			if(i%4 != 3) {
				matPrevious.ptr(j)[i] = ppcImgPrevious[j*WIDTH*4+m];
				matActive.ptr(j)[i] = ppcImgActive[j*WIDTH*4+m];
				++m;
			}
			else {
				matActive.ptr(j)[i] = ppcImgActive[j*WIDTH*4+i+WIDTH*3];
			}
			++i;
		}
		i=0;
		j++;
		m=0;
	}


	// Visualise the data
	namedWindow("Image", 1);
    imshow("Image", matPrevious);
	waitKey();

	// Free memory
	if(is_FreeImageMem(cameraHandle, ppcImgActive, pidActive) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreeImageMem");
	if(is_FreeImageMem(cameraHandle, ppcImgPrevious, pidPrevious) != IS_SUCCESS)
		spawn_error(cameraHandle, "is_FreeImageMem");

	is_ExitCamera(cameraHandle);
	return 0;
}