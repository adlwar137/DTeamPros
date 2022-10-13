#pragma once
#include "main.h"

typedef struct chassis {
  uint8_t frontLeftMotor;
  uint8_t frontRightMotor;
  uint8_t backLeftMotor;
  uint8_t backRightMotor;
} chassis_t;

int32_t base_move(chassis_t chassis, const int32_t x_voltage, const int32_t y_voltage, const int32_t w_voltage);

int32_t base_move_velocity(chassis_t chassis, int32_t x_velocity, int32_t y_velocity, int32_t w_velocity);

int32_t base_brake(chassis_t chassis);

int32_t base_set_brake_mode(chassis_t chassis, const pros::motor_brake_mode_e_t mode);

int32_t base_set_gearing(chassis_t chassis, const pros::motor_gearset_e gearset);

int32_t base_set_reversed(chassis_t chassis, const bool reversed);
