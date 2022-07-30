#include "chassis.h"

using namespace pros::c;

int32_t base_move(chassis_t chassis, const int32_t x_voltage, const int32_t y_voltage) {
	motor_move(chassis.frontLeftMotor, y_voltage + x_voltage);
	motor_move(chassis.frontRightMotor, y_voltage - x_voltage);
	motor_move(chassis.backLeftMotor, y_voltage - x_voltage);
	motor_move(chassis.backRightMotor, y_voltage + x_voltage);	
	return 1;
}

int32_t base_move_velocity(chassis_t chassis, const int32_t x_velocity, const int32_t y_velocity) {
	motor_move_velocity(chassis.frontLeftMotor, y_velocity + x_velocity);
	motor_move_velocity(chassis.frontRightMotor, y_velocity - x_velocity);
	motor_move_velocity(chassis.backLeftMotor, y_velocity - x_velocity);
	motor_move_velocity(chassis.backRightMotor, y_velocity + x_velocity);	
	return 1;
}

int32_t base_turn(chassis_t chassis, const int32_t x_voltage) {
	motor_move(chassis.frontLeftMotor, x_voltage);
	motor_move(chassis.frontRightMotor, -x_voltage);
	motor_move(chassis.backLeftMotor, x_voltage);
	motor_move(chassis.backRightMotor, -x_voltage);
	return 1;
}

int32_t base_turn_velocity(chassis_t chassis, const int32_t x_voltage) {
	motor_move_velocity(chassis.frontLeftMotor, x_voltage);
	motor_move_velocity(chassis.frontRightMotor, -x_voltage);
	motor_move_velocity(chassis.backLeftMotor, x_voltage);
	motor_move_velocity(chassis.backRightMotor, -x_voltage);
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