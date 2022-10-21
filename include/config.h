#pragma once
#include "main.h"
#include "odometrySensor.h"
#include "flywheel.h"
#include "chassis.h"

#define TRACKING_WHEEL_DIAMETER 2.75
#define LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4
#define RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4
#define STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4.75
#define POWERED_WHEEL_DISTANCE_FROM_CENTER 8.25

extern pros::Motor frontLeft;
extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor backRight;

extern pros::Motor flywheelLeft;
extern pros::Motor flywheelRight;

extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder strafe_encoder;

extern Chassis base;
extern Flywheel discShooter;
extern OdometrySensor odometry;

//global pose
extern vector3d_t pose;