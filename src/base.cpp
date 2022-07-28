#include "base.h"

using namespace pros::c;

int32_t base_move(chassis Chassis, const int8_t x, const int8_t y) {
	motor_move(Chassis.frontLeftMotor, y + x);
	motor_move(Chassis.frontRightMotor, y - x);
	motor_move(Chassis.backLeftMotor, y - x);
	motor_move(Chassis.backRightMotor, y + x);	
	return 1;
}

int32_t base_turn(chassis Chassis, const int8_t x) {
	motor_move(Chassis.frontLeftMotor, x);
	motor_move(Chassis.frontRightMotor, -x);
	motor_move(Chassis.backLeftMotor, x);
	motor_move(Chassis.backRightMotor, -x);
	return 1;
}

int32_t base_set_gearing(chassis Chassis, const pros::motor_gearset_e gearset) {
	motor_set_gearing(Chassis.frontLeftMotor, gearset);
	motor_set_gearing(Chassis.frontRightMotor, gearset);
	motor_set_gearing(Chassis.backLeftMotor, gearset);
	motor_set_gearing(Chassis.backRightMotor, gearset);
	return 1;
}

int32_t base_set_reversed(chassis Chassis, const bool reversed) {
	motor_set_reversed(Chassis.frontLeftMotor, reversed);
	motor_set_reversed(Chassis.frontRightMotor, reversed);
	motor_set_reversed(Chassis.backLeftMotor, reversed);
	motor_set_reversed(Chassis.backRightMotor, reversed);
	return 1;
}