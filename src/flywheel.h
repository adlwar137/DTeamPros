#pragma once
#include "main.h"
#include "PIDController.h"
#include "constants.h"
#include "mathy.h"

enum flywheelMode{Auton, Driver};
class Flywheel {
  public:
    Flywheel(pros::Motor* leftMotor,
             pros::Motor* rightMotor);
    
    Flywheel(pros::Motor* leftMotor,
             pros::Motor* rightMotor,
             pros::motor_gearset_e_t gearset);

    void set_gearing(pros::motor_gearset_e_t gearing);

    void set_brake_mode(pros::motor_brake_mode_e_t brakemode);

    double get_actual_average_velocity();

    void spin(int32_t voltage);
    
    void spin_velocity(double velocity);

    void update();

    void brake();
  private:
    
    double desired_velocity;
    flywheelMode mode;
    PIDController* left_flywheel_pid;
    PIDController* right_flywheel_pid;
    pros::Motor* leftMotor;
    pros::Motor* rightMotor;
    pros::motor_gearset_e_t gearset;
};