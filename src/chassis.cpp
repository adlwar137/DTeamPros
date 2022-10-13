#include "chassis.h"

using namespace pros::c;

int32_t base_move(chassis_t chassis, const int32_t x_voltage, const int32_t y_voltage, const int32_t w_voltage) {
  motor_move(chassis.frontLeftMotor, y_voltage + x_voltage + w_voltage);
  motor_move(chassis.frontRightMotor, y_voltage - x_voltage - w_voltage);
  motor_move(chassis.backLeftMotor, y_voltage - x_voltage + w_voltage);
  motor_move(chassis.backRightMotor, y_voltage + x_voltage - w_voltage);
  return 1;
}

int32_t base_move_velocity(chassis_t chassis, int32_t x_velocity, int32_t y_velocity, int32_t w_velocity) {
  if((x_velocity + y_velocity + w_velocity) > 200) {
    double multi = (double)200/(x_velocity+y_velocity+w_velocity);
    x_velocity = x_velocity * multi;
    y_velocity = y_velocity * multi;
    w_velocity = w_velocity * multi;
  }
  motor_move_velocity(chassis.frontLeftMotor, y_velocity + x_velocity + w_velocity);
  motor_move_velocity(chassis.frontRightMotor, y_velocity - x_velocity - w_velocity);
  motor_move_velocity(chassis.backLeftMotor, y_velocity - x_velocity + w_velocity);
  motor_move_velocity(chassis.backRightMotor, y_velocity + x_velocity - w_velocity);
  return 1;
}

int32_t base_brake(chassis_t chassis) {
  motor_brake(chassis.frontLeftMotor);
  motor_brake(chassis.frontRightMotor);
  motor_brake(chassis.backLeftMotor);
  motor_brake(chassis.backRightMotor);
  return 1;
}

int32_t base_set_brake_mode(chassis_t chassis, pros::motor_brake_mode_e_t mode) {
  motor_set_brake_mode(chassis.frontLeftMotor, mode);
  motor_set_brake_mode(chassis.frontRightMotor, mode);
  motor_set_brake_mode(chassis.backLeftMotor, mode);
  motor_set_brake_mode(chassis.backRightMotor, mode);
  return 1;
}

int32_t base_set_gearing(chassis_t chassis, const pros::motor_gearset_e gearset) {
  motor_set_gearing(chassis.frontLeftMotor, gearset);
  motor_set_gearing(chassis.frontRightMotor, gearset);
  motor_set_gearing(chassis.backLeftMotor, gearset);
  motor_set_gearing(chassis.backRightMotor, gearset);
  return 1;
}

int32_t base_set_reversed(chassis_t chassis, const bool reversed) {
  motor_set_reversed(chassis.frontLeftMotor, reversed);
  motor_set_reversed(chassis.frontRightMotor, reversed);
  motor_set_reversed(chassis.backLeftMotor, reversed);
  motor_set_reversed(chassis.backRightMotor, reversed);
  return 1;
}
