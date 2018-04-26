#include <cstdio>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"

int main(int argc, char *argv[])
{
	//auto sampling=std::chrono::milliseconds(10000);
	std::ofstream my_file;
	my_file.open("LPMS_DATA.txt");	
	ImuData d;

	// Gets a LpmsSensorManager instance
	LpmsSensorManagerI* manager = LpmsSensorManagerFactory();

	// Connects to LPMS-B sensor with address 00:11:22:33:44:55 
	LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_U, "A1019SCB");

	//UART BAUDRATE
	int a=100;
	lpms->getConfigurationPrm(41, &a);
	std::cout << "a_before: " << a << std::endl;
	lpms->setConfigurationPrm(41, 3);
	lpms->getConfigurationPrm(41, &a);
	std::cout << "a_after: " << a << std::endl;

	auto beginning = std::chrono::high_resolution_clock::now();

	while(1) {
		getchar();
		// Checks, if conncted
		if (
			lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
			lpms->hasImuData()
			) {

			auto timestamp = std::chrono::high_resolution_clock::now();
			// Reads quaternion data
			d = lpms->getCurrentData();

			printf("%f %f %f\n%f %f %f\n%f %f %f\n", 
				d.rotationM[0], d.rotationM[1],d.rotationM[2],
				d.rotationM[3],d.rotationM[4], d.rotationM[5], 
				d.rotationM[6], d.rotationM[7], d.rotationM[8]);
/*
			my_file << std::chrono::duration_cast<std::chrono::milliseconds>(timestamp-beginning).count() <<
			" " << d.q[0] << " " << " " << d.q[1] << " " << " " << d.q[2] << " " <<
			" " << d.q[3] << std::endl;

			std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(timestamp-beginning).count() <<
			" " << d.q[0] << " " << " " << d.q[1] << " " << " " << d.q[2] << " " <<
			" " << d.q[3] << std::endl;
*/
			// Shows data
			//printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n", 
				//d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3]);
			//printf("Timestamp:%f\n %f %f %f\n%f %f %f\n%f %f %f\n", 
			//	d.timeStamp, d.rotationM[0], d.rotationM[1],d.rotationM[2],d.rotationM[3],d.rotationM[4],
			//		d.rotationM[5], d.rotationM[6], d.rotationM[7], d.rotationM[8]);
			//std::cout << "Tempo:" << std::chrono::duration_cast<std::chrono::milliseconds>(end-beginning).count() << std::endl;
			
		}
		//std::cout << "Tempo:" << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count() << std::endl;
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));

		//auto started2 = std::chrono::high_resolution_clock::now();	
		//std::this_thread::sleep_for(std::chrono::milliseconds(sampling-interval));
		//auto done2 = std::chrono::high_resolution_clock::now();
		//std::cout << "Tempo_wait:" << std::chrono::duration_cast<std::chrono::milliseconds>(done2-started2).count() << std::endl;
	}

	// Removes the initialized sensor
	manager->removeSensor(lpms);
	
	my_file.close();	
	// Deletes LpmsSensorManager object 
	delete manager;
	
	return 0;
}
