#include "flywheel.h"

Flywheel::Flywheel(pros::Motor* leftMotor, pros::Motor* rightMotor, pros::Rotation* leftRotation, pros::Rotation* rightRotation) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->leftRotation = leftRotation;
    this->rightRotation = rightRotation;
    this->left_flywheel_pid = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    this->right_flywheel_pid = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    this->mode = flywheelMode::Driver;
}

Flywheel::Flywheel(pros::Motor* leftMotor, pros::Motor* rightMotor, pros::Rotation* leftRotation, pros::Rotation* rightRotation, pros::motor_gearset_e_t gearset) : Flywheel(leftMotor, rightMotor, leftRotation, rightRotation) {
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

void Flywheel::set_mode(flywheelMode mode) {
    this->mode = mode;
}

//TODO update this to use the new v5 rotation sensors
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
            double left_voltage = this->left_flywheel_pid->calculate(desired_velocity - ((double)(this->leftRotation->get_velocity()*60/360)));
            double right_voltage = this->right_flywheel_pid->calculate(desired_velocity + ((double)(this->rightRotation->get_velocity()*60/360)));

        /*
            printf("leftFlywheelEncoder: %f, left_voltage: %f ", (double)this->leftRotation->get_velocity()*60/360, left_voltage);
            printf("rightFlywheelEncoder: %f, right_voltage: %f\n", (double)this->rightRotation->get_velocity()*60/360, right_voltage);
        */

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