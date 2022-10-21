#include "locationSensor.h"

LocationSensor::LocationSensor(GpsArray* gpsArray, pros::Imu* inertial) {
    this->gpsArray = gpsArray;
    this->inertial = inertial;
}