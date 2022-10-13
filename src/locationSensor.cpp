#include "locationSensor.h"

using namespace pros::c;

typedef struct sensorArray {
    uint8_t leftGPS;
    uint8_t rightGPS;
} sensorArray_t;

double locationSensor_get_heading() {
    return (gps_get_heading(GPS_LEFT) + gps_get_heading(GPS_RIGHT)) / 2;
}



vector3d_t locationSensor_get_pose() {
    vector3d_t pose;
    pose.w = locationSensor_get_heading();
    pose.x = (gps_get_status(GPS_LEFT).x*39.37)+5;
    pose.y = (gps_get_status(GPS_LEFT).y*39.37);
    return pose;
}