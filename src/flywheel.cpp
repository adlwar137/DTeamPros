#include "flywheel.h"

Flywheel::Flywheel(pros::Motor* leftMotor, pros::Motor* rightMotor) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
}

Flywheel::Flywheel(pros::Motor* leftMotor, pros::Motor* rightMotor, pros::motor_gearset_e_t gearset) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    Flywheel::set_gearing(gearset);
}

int32_t Flywheel::set_gearing(pros::motor_gearset_e_t gearset) {
    this->leftMotor->set_gearing(gearset);
    this->rightMotor->set_gearing(gearset);
    this->gearset = gearset;
    return 0;
}

int32_t Flywheel::set_brake_mode(pros::motor_brake_mode_e_t brakemode) {
    this->leftMotor->set_brake_mode(brakemode);
    this->rightMotor->set_brake_mode(brakemode);
    return 0;
}

double Flywheel::get_actual_average_velocity() {
    double average = (this->leftMotor->get_actual_velocity() + this->rightMotor->get_actual_velocity()) / 2;
    return average;
}

int32_t Flywheel::spin(int32_t voltage) {
    this->leftMotor->move(voltage);
    this->rightMotor->move(voltage);
    return 0;
}

int32_t Flywheel::brake() {
    this->leftMotor->brake();
    this->rightMotor->brake();
    return 0;
}