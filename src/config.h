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

extern pros::Rotation flywheelRotationLeft;
extern pros::Rotation flywheelRotationRight;

extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder strafe_encoder;

extern pros::Imu inertial;

extern pros::Vision rollerSensor;

extern pros::vision_signature RED_;

extern pros::ADIDigitalOut piston;
extern pros::ADIDigitalOut rollerLED;

extern pros::ADIUltrasonic rangeFinder;

extern pros::ADIButton intakeLimit;

extern Chassis base;
extern Flywheel flywheel;
extern OdometrySensor odometry;