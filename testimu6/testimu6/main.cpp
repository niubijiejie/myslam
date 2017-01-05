#include <iostream>

#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"



using namespace std;

int main()
{
    ImuData d;

    // Gets a LpmsSensorManager instance
    LpmsSensorManagerI* manager = LpmsSensorManagerFactory();

    // Connects to LPMS-B sensor with address 00:11:22:33:44:55
    LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_U, "A5022WD0");

    while(1) {
        // Checks, if conncted
        if (
            lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
            lpms->hasImuData()
            ) {

            // Reads quaternion data
            d = lpms->getCurrentData();

            // Shows data
            printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
                d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3]);
        }
    }

    // Removes the initialized sensor
    manager->removeSensor(lpms);

    // Deletes LpmsSensorManager object
    delete manager;

    return 0;

}
