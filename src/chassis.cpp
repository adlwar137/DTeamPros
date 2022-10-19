#include "chassis.h"

using namespace pros::c;

Chassis::Chassis(pros::Motor* frontLeftMotor, pros::Motor* frontRightMotor, pros::Motor* backLeftMotor, pros::Motor* backRightMotor) {
  this->frontLeftMotor = frontLeftMotor;
  this->frontRightMotor = frontRightMotor;
  this->backLeftMotor = backLeftMotor;
  this->backRightMotor = backRightMotor;
  //default to green
  Chassis::set_gearing(pros::motor_gearset_e_t::E_MOTOR_GEARSET_18);
}

Chassis::Chassis(pros::Motor* frontLeftMotor, pros::Motor* frontRightMotor, pros::Motor* backLeftMotor, pros::Motor* backRightMotor, pros::motor_gearset_e_t gearset) {
  this->frontLeftMotor = frontLeftMotor;
  this->frontRightMotor = frontRightMotor;
  this->backLeftMotor = backLeftMotor;
  this->backRightMotor = backRightMotor;
  Chassis::set_gearing(gearset);
}

int32_t Chassis::set_gearing(pros::motor_gearset_e_t gearset) {
  this->frontLeftMotor->set_gearing(gearset);
  this->frontRightMotor->set_gearing(gearset);
  this->backLeftMotor->set_gearing(gearset);
  this->backRightMotor->set_gearing(gearset);
  this->gearset = gearset;
  return 0;
}

int32_t Chassis::set_brake_mode(pros::motor_brake_mode_e_t brakemode) {
  this->frontLeftMotor->set_brake_mode(brakemode);
  this->frontRightMotor->set_brake_mode(brakemode);
  this->backLeftMotor->set_brake_mode(brakemode);
  this->backRightMotor->set_brake_mode(brakemode);
  return 0;
}

int32_t Chassis::move(int32_t x_voltage, int32_t y_voltage, int32_t w_voltage) {
  this->frontLeftMotor->move(y_voltage + x_voltage + w_voltage);
  this->frontRightMotor->move(y_voltage - x_voltage - w_voltage);
  this->backLeftMotor->move(y_voltage - x_voltage + w_voltage);
  this->backRightMotor->move(y_voltage + x_voltage - w_voltage);
  return 0;
}

int32_t Chassis::move_velocity(int32_t x_velocity, int32_t y_velocity, int32_t w_velocity) {
  this->frontLeftMotor->move_velocity(y_velocity + x_velocity + w_velocity);
  this->frontRightMotor->move_velocity(y_velocity - x_velocity - w_velocity);
  this->backLeftMotor->move_velocity(y_velocity - x_velocity + w_velocity);
  this->backRightMotor->move_velocity(y_velocity + x_velocity - w_velocity);
  return 0;
}

//takes -1 to 1 values on all params
int32_t Chassis::move_vector(double angle, double power, double turn) {
  double SIN = sin(angle - M_PI/4);
  double COS = cos(angle - M_PI/4);
  double max = mathy_max(fabs(SIN), fabs(COS));

  double frontLeftVelocity = power * COS/max + turn;
  double frontRightVelocity = power * SIN/max - turn;
  double backLeftVelocity = power * SIN/max + turn;
  double backRightVelocity = power * COS/max - turn;

  if((power + fabs(turn)) < -1) {
    frontLeftVelocity /= power + turn;
    frontRightVelocity /= power + turn;
    backLeftVelocity /= power + turn;
    backRightVelocity /= power + turn;
  }

  this->frontLeftMotor->move_velocity(frontLeftVelocity * Chassis::get_max_speed());
  this->frontRightMotor->move_velocity(frontRightVelocity * Chassis::get_max_speed());
  this->backLeftMotor->move_velocity(backLeftVelocity * Chassis::get_max_speed());
  this->backRightMotor->move_velocity(backRightVelocity * Chassis::get_max_speed());
  return 0;
}

int32_t Chassis::brake() {
  this->frontLeftMotor->brake();
  this->frontRightMotor->brake();
  this->backLeftMotor->brake();
  this->backRightMotor->brake();
  return 0;
}

int32_t Chassis::get_max_speed() {
  switch (this->gearset) {
    case pros::E_MOTOR_GEARSET_06:
      return 600;
    break;
    case pros::E_MOTOR_GEARSET_18:
      return 200;
    break;
    case pros::E_MOTOR_GEARSET_36:
      return 100;
    break;
  }
  return 0;
}