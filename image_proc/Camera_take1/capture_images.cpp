// Compile with  g++ -std=c++11 capture_images.cpp -lueye_api `pkg-config --libs --cflags opencv` -o capture

#include "defs.hpp"
#include <ueye.h>
#include <wchar.h>


#define HEIGHT 1542
#define WIDTH 2056
#define BITSPIXEL 32
#define N_IMAGES 65
#define PATH "/home/imarcher/image"

void spawn_error(int cameraHandle, std::string where) {
	char *ppcErr;
	int pErr = 0;
	is_GetError(cameraHandle, &pErr, &ppcErr);
 	std::cout << "Error at " << where << ppcErr << std::endl;
 	//exit(EXIT_FAILURE);
}

int main( int argc, char** argv ) {

	char *img = NULL;
	int pid;
	int counter = 0;
	Mat matImg(HEIGHT, WIDTH, CV_8UC4);

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
		cout << "Camera is connected" << std::endl;

/*
	wchar_t filepath[30];
	IMAGE_FILE_PARAMS ImageFileParams;
    ImageFileParams.pnImageID = NULL;
    ImageFileParams.ppcImageMem = NULL;
    ImageFileParams.nQuality = 100;
    ImageFileParams.nFileType = IS_IMG_PNG;
*/
	int j, i, m;
	ostringstream imagePath;
	while(counter < N_IMAGES) {

		// Allocate memory for image
		if(is_AllocImageMem(cameraHandle, WIDTH, HEIGHT, BITSPIXEL, &img, &pid) != IS_SUCCESS) 
			spawn_error(cameraHandle, "is_AllocImageMem");
		// Set image memory for image
		if(is_SetImageMem(cameraHandle, img, pid) != IS_SUCCESS) 
			spawn_error(cameraHandle, "is_SetImageMem");

		// Capture image 
		if(is_FreezeVideo(cameraHandle, IS_WAIT) != IS_SUCCESS)
			spawn_error(cameraHandle, "is_FreezeVideo");
		else {
			/*
			memset(filepath, 0, sizeof(filepath));
			swprintf(filepath, 22, L"/home/imarcher/image%d", counter);
			ImageFileParams.pwchFileName = filepath;

			if(is_ImageFile(cameraHandle, IS_IMAGE_FILE_CMD_SAVE, (void*)&ImageFileParams, sizeof(ImageFileParams))!= IS_SUCCESS) 
				spawn_error(cameraHandle, "is_ImageFile");
			else {
				cout << "Image " << to_string(counter) << " was captured!" << std::endl;
				counter++;
			}
			*/
			// Convert the image in memory to an OpenCV Mat variable
			j = 0, i = 0, m = 0;
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

			imagePath << PATH << counter << ".bmp";
			imwrite(imagePath.str(), matImg);
			cout << "Image " << imagePath.str() << " was captured!" << std::endl;
			counter++;
			imagePath.str("");
		}
		// Free memory
		if(is_FreeImageMem(cameraHandle, img, pid) != IS_SUCCESS)
			spawn_error(cameraHandle, "is_FreeImageMem");

		getchar();
	}
	

	is_ExitCamera(cameraHandle);
	return 0;
}

