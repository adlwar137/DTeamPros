#pragma once
#include "main.h"
#include "odometrySensor.h"
#include "flywheel.h"
#include "chassis.h"

extern pros::Motor frontLeft;
extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor backRight;

extern pros::Motor flywheelLeft;
extern pros::Motor flywheelRight;

extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder strafe_encoder;

extern pros::Imu inertial;

extern Chassis base;
extern Flywheel discShooter;
extern OdometrySensor odometry;

//global pose
extern vector3d_t pose;