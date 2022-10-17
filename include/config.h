#pragma once
#include "main.h"
#include "odometry.h"
#include "flywheel.h"
#include "chassis.h"

#define TRACKING_WHEEL_DIAMETER 2.75
#define LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 3.75
#define RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 3.5
#define STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4.75
#define POWERED_WHEEL_DISTANCE_FROM_CENTER 8.25

extern pros::Motor frontLeft;
extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor backRight;

extern pros::Motor flywheelLeft;
extern pros::Motor flywheelRight;

extern Chassis base;
extern Flywheel discShooter;

const int32_t MOTOR_MAX_VOLTAGE = 127;
const int32_t MOTOR_MIN_VOLTAGE = -127;

const uint8_t INTAKE = 10;

const uint8_t PUNCHER = 9;

const uint8_t LED = 8;

const uint8_t ADI_ENCODER_RIGHT_TOP = 1;
const uint8_t ADI_ENCODER_RIGHT_BOTTOM = 2;

const uint8_t ADI_ENCODER_LEFT_TOP = 3;
const uint8_t ADI_ENCODER_LEFT_BOTTOM = 4;

const uint8_t ADI_ENCODER_STRAFE_TOP = 5;
const uint8_t ADI_ENCODER_STRAFE_BOTTOM = 6;

const uint8_t GPS_LEFT = 12;
const uint8_t GPS_RIGHT = 11;

const pros::controller_id_e_t MASTER_CONTROLLER = pros::controller_id_e_t::E_CONTROLLER_MASTER;

extern pros::c::adi_encoder_t right_encoder;
extern pros::c::adi_encoder_t left_encoder;
extern pros::c::adi_encoder_t strafe_encoder;

//global pose
extern vector3d_t pose;