#include "sensor.h"
#include "defs.h"

void ThrowError(string what) {
    cout << RED << "(main.cpp) ERROR : " << RESET << what << endl;
    exit(EXIT_FAILURE);
}

int main() {

	Sensor mySensor;
	bool ret;

	// Connect sensor
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;

    while(1) {	

		// Get orientation 
	    ret = mySensor.GetOrientation();
	    if (!ret) ThrowError("Sensor was not able to get orientation");
	    cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
	    cout << BLUE << "IMU (eul)" << RESET << mySensor.eul << endl;
	    cout << BLUE << "IMU (quat)" << RESET << mySensor.eul << endl;
	    getchar();

    }

}