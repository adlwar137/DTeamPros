#pragma once
#include "main.h"
#include "flywheel.h"
#include "chassis.h"
#include "odometry.h"

using namespace pros::c;

const int32_t MOTOR_MAX_VOLTAGE = 127;
const int32_t MOTOR_MIN_VOLTAGE = -127;

const uint8_t FRONTRIGHT = 1;
const uint8_t BACKRIGHT = 2;
const uint8_t BACKLEFT = 3;
const uint8_t FRONTLEFT = 4;

const uint8_t FLYWHEELA = 5;
const uint8_t FLYWHEELB = 7;

const uint8_t INTAKE = 10;

const uint8_t PUNCHER = 9;

const uint8_t PISTON = 8;

const uint8_t ADI_ENCODER_RIGHT_TOP = 1;
const uint8_t ADI_ENCODER_RIGHT_BOTTOM = 2;

const uint8_t ADI_ENCODER_LEFT_TOP = 3;
const uint8_t ADI_ENCODER_LEFT_BOTTOM = 4;

const uint8_t ADI_ENCODER_STRAFE_TOP = 5;
const uint8_t ADI_ENCODER_STRAFE_BOTTOM = 6;

const pros::controller_id_e_t MASTER_CONTROLLER = pros::controller_id_e_t::E_CONTROLLER_MASTER;

extern adi_encoder_t right_encoder;
extern adi_encoder_t left_encoder;
extern adi_encoder_t strafe_encoder;

extern chassis_t base;
extern flywheel discShooter;

// odometry task paramaters
extern tracking_params_t params;

//global pose
extern vector3d pose;