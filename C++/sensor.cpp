#include "sensor.h"

Sensor::Sensor() {
    manager = LpmsSensorManagerFactory();
    rotm = Mat::zeros(cv::Size(3,3), DataType<double>::type);
    quat = Mat::zeros(cv::Size(3,1), DataType<double>::type);
    eul = Mat::zeros(cv::Size(4,3), DataType<double>::type);
}

bool Sensor::Connect(const char* _id, int _type) {
    sensor = manager->addSensor(_type, _id);
    return sensor->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED;
}

bool Sensor::GetOrientation() {
    if(sensor->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED)
        return false;
    if(!sensor->hasImuData())
        return false;
    data = sensor->getCurrentData();

    int count = 0;
    for(int i = 1; i < 3; i++) {
        for(int j = 1; j < 3; j++) {
            rotm.at<double>(i,j) = data.rotationM[count];
            count ++;
        }
    }
    for(int i = 1; i < 4; i++) {
        quat.at<double>(i) = data.q[i];
    }
    for(int i = 1; i < 3; i++) {
        eul.at<double>(i) = data.r[i];
    }
    timestamp = chrono::high_resolution_clock::now();
    return true;
}

void Sensor::Disconnect() {
    manager->removeSensor(sensor);
    delete manager;
}

