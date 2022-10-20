#pragma once
#include "main.h"
#include "odometry.h"
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

const int32_t MOTOR_MAX_VOLTAGE = 127;
const int32_t MOTOR_MIN_VOLTAGE = -127;

const uint8_t INTAKE = 10;

const uint8_t PUNCHER = 9;

const uint8_t LED = 8;

const uint8_t GPS_LEFT = 12;
const uint8_t GPS_RIGHT = 11;

const pros::controller_id_e_t MASTER_CONTROLLER = pros::controller_id_e_t::E_CONTROLLER_MASTER;

//global pose
extern vector3d_t pose;