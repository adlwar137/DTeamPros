#pragma once
#include "main.h"
#include "mathy.h"

class Chassis {
  public:
    Chassis(pros::Motor* frontLeftMotor, 
            pros::Motor* frontRightMotor,
            pros::Motor* backLeftMotor,
            pros::Motor* backRightMotor);

    Chassis(pros::Motor* frontLeftMotor,
            pros::Motor* frontRightMotor,
            pros::Motor* backLeftMotor,
            pros::Motor* backRightMotor,
            pros::motor_gearset_e_t gearset);

    int32_t set_gearing(pros::motor_gearset_e_t gearset);

    int32_t set_brake_mode(pros::motor_brake_mode_e_t brakemode);

    int32_t move(int32_t x_voltage, int32_t y_voltage, int32_t w_voltage);

    int32_t move_velocity(int32_t x_velocity, int32_t y_velocity, int32_t w_velocity);

    int32_t move_vector(double angle, double power, double turn);

    int32_t brake();
  private:
    pros::Motor* frontLeftMotor;
    pros::Motor* frontRightMotor;
    pros::Motor* backLeftMotor;
    pros::Motor* backRightMotor;
    pros::motor_gearset_e_t gearset;

    int32_t get_max_speed();
};