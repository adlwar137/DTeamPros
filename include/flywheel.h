#pragma once
#include "main.h"

class Flywheel {
  public:
    Flywheel(pros::Motor* leftMotor,
             pros::Motor* rightMotor);
    
    Flywheel(pros::Motor* leftMotor,
             pros::Motor* rightMotor,
             pros::motor_gearset_e_t gearset);

  int32_t set_gearing(pros::motor_gearset_e_t gearing);

  int32_t set_brake_mode(pros::motor_brake_mode_e_t brakemode);

  double get_actual_average_velocity();

  int32_t spin(int32_t voltage);

  int32_t brake();
  private:
    pros::Motor* leftMotor;
    pros::Motor* rightMotor;
    pros::motor_gearset_e_t gearset;
};