// Compile with g++ -Wall get_image.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lueye_api -o get_image

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ueye.h>
#include <iostream>

using namespace cv;

#define HEIGHT 480
#define WIDTH 752
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
	//int col = 0, inc1 = 0, inc2 = 0, row = 0;
/*
	while(row < 480) {

		while(col < 752*4*(row+1)) {
			if(col < 752*3*(row+1)) {
				matPrevious.ptr(row)[inc1] = 255;//ppcImgPrevious[col];
				col++;
				matPrevious.ptr(row)[inc1+752] = 255;//ppcImgPrevious[col];
				col++;
				matPrevious.ptr(row)[inc1+752*2] = 255;//ppcImgPrevious[col];
				col++;	
				inc1++;
			}
			else {
				matPrevious.ptr(row)[inc2+752*3] = 0;//ppcImgPrevious[col];
				col++;
				inc2++;
			}
		}

		inc1 = 0; inc2 = 0;
		row++;
	}
*/

	int j = 0, i = 0, m = 0;
	while(j < 480) {
		while(i<752*4)
		{
			if(i%4 != 3) {
				matPrevious.ptr(j)[i] = ppcImgPrevious[j*752*4+m];
				++m;
			}
			else {
				matPrevious.ptr(j)[i] = ppcImgPrevious[j*752*4+i+752*3];

			}
			++i;
		}
		i=0;
		j++;
		m=0;
	}

	for (int l = 0; l < 752*3; ++l) {
		printf("%d ", ppcImgPrevious[l]);
		
	}
	printf("\n\n\n\n\n");
	for (int l = 752*3; l < 752*4; ++l) {
		printf("%d ", ppcImgPrevious[l]);
		
	}
	printf("\n\n\n\n\n");
	for (int l = 0; l < 752*4; ++l) {
		printf("%d ", matPrevious.ptr(0)[l]);

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