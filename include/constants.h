#pragma once
#include "main.h"

#define TRACKING_WHEEL_DIAMETER 2.75
#define LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4
#define RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4
#define STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4.75
#define POWERED_WHEEL_DISTANCE_FROM_CENTER 8.25

const int32_t MOTOR_MAX_VOLTAGE = 127;
const int32_t MOTOR_MIN_VOLTAGE = -127;

const uint8_t INTAKE = 10;

const uint8_t PUNCHER = 9;

const uint8_t LED = 8;

const uint8_t GPS_LEFT = 12;
const uint8_t GPS_RIGHT = 11;

const pros::controller_id_e_t MASTER_CONTROLLER = pros::controller_id_e_t::E_CONTROLLER_MASTER;
