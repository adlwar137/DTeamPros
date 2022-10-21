#pragma once
#include "main.h"
#include "config.h"
#include "mathy.h"
#include "gpsArray.h"

//I have a gps sensor which gives me decently accurate readings whenever its working
//There are two of them ^
//I have an inertial sensor which gives me very accurate heading although it takes time to calibrate
//I have 3 wheeled odometry which is very good for low latency position (not so much heading though, the wheel drift on left and right is getting pretty bad)

//two options:
//I should have a "global" pose struct which tells me my position and heading
//I give a location Sensor class methods to get status or heading or position like the gps sensor

//I think the location sensor class with methods makes more sense

class LocationSensor {
    public:
        LocationSensor(GpsArray* gpsArray, pros::Imu* inertial);
    private:
        GpsArray* gpsArray;
        pros::Imu* inertial;
        vector3d_t pose;
};