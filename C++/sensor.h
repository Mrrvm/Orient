#ifndef MAIN_CPP_SENSOR_H
#define MAIN_CPP_SENSOR_H

#include "defs.h"
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"


class Sensor {

    LpmsSensorManagerI* manager;
    LpmsSensorI* sensor;
    ImuData data;

public:
    Mat rotm;
    Mat eul;
    Mat quat;
    Mat angaxis;
    chrono::high_resolution_clock::time_point timestamp;

    Sensor();
    bool Connect(const char* , int);
    bool GetOrientation();
    void Disconnect();
};


#endif //MAIN_CPP_SENSOR_H
