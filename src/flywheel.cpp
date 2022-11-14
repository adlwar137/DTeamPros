#include "flywheel.h"

Flywheel::Flywheel(pros::Motor* leftMotor, pros::Motor* rightMotor) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->left_flywheel_pid = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, 12000 / FLYWHEEL_KI);
    this->right_flywheel_pid = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, 12000 / FLYWHEEL_KI);
    this->mode = flywheelMode::Driver;
}

Flywheel::Flywheel(pros::Motor* leftMotor, pros::Motor* rightMotor, pros::motor_gearset_e_t gearset) : Flywheel(leftMotor, rightMotor) {
    Flywheel::set_gearing(gearset);
}

void Flywheel::set_gearing(pros::motor_gearset_e_t gearset) {
    this->leftMotor->set_gearing(gearset);
    this->rightMotor->set_gearing(gearset);
    this->gearset = gearset;
}

void Flywheel::set_brake_mode(pros::motor_brake_mode_e_t brakemode) {
    this->leftMotor->set_brake_mode(brakemode);
    this->rightMotor->set_brake_mode(brakemode);
}

void Flywheel::set_flywheel_mode(flywheelMode mode) {
    this->mode = mode;
}

double Flywheel::get_actual_average_velocity() {
    double average = (this->leftMotor->get_actual_velocity() + this->rightMotor->get_actual_velocity()) / 2;
    return average;
}

void Flywheel::spin(int32_t voltage) {
    this->leftMotor->move(voltage);
    this->rightMotor->move(voltage);
}

void Flywheel::spin_velocity(double velocity) {
    this->desired_velocity = velocity;
}

void Flywheel::update() {
    if(this->mode == flywheelMode::Auton) {
            double left_voltage = this->left_flywheel_pid->calculate(desired_velocity - this->leftMotor->get_actual_velocity());
            double right_voltage = this->right_flywheel_pid->calculate(desired_velocity - this->rightMotor->get_actual_velocity());

            this->leftMotor->move_voltage(left_voltage);
            this->rightMotor->move_voltage(right_voltage);
    } else {
        //bro what?
    }
}

void Flywheel::brake() {
    this->leftMotor->brake();
    this->rightMotor->brake();
}