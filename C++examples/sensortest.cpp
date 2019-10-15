#include "sensor.h"
#include "defs.h"

void ThrowError(string what) {
    cout << RED << "(main.cpp) ERROR : " << RESET << what << endl;
    exit(EXIT_FAILURE);
}

int main() {

    Sensor mySensor;
    bool ret;
    Mat quat2, quat1;
    Mat peul, eul;

    // Connect sensor
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;

    eul.create(1, 3, DataType<double>::type);
    ret = mySensor.GetOrientation();
    mySensor.quat.copyTo(quat1);

    while(1) {

        ret = mySensor.GetOrientation();
        if (!ret) ThrowError("Sensor was not able to get orientation");
        cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
        mySensor.quat.copyTo(quat2);
        peul = Quat2Eul(QuatMultiply(quat2, QuatConjugate(quat1)));
        eul.at<double>(1) = -peul.at<double>(0);
        eul.at<double>(2) = peul.at<double>(1);
        eul.at<double>(0) = peul.at<double>(2);
        cout << BLUE << "IMU " << RESET << eul*180/M_PI << endl;

        quat2.copyTo(quat1);
        getchar();
    }

}