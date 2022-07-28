#include "main.h"

typedef struct chassis {
	uint8_t frontLeftMotor;	
	uint8_t frontRightMotor;
	uint8_t backLeftMotor;
	uint8_t backRightMotor;
} chassis;

int32_t base_move(chassis Chassis, const int8_t x, const int8_t y);

int32_t base_turn(chassis Chassis, const int8_t x);

int32_t base_set_gearing(chassis Chassis, const pros::motor_gearset_e gearset);

int32_t base_set_reversed(chassis Chassis, const bool reversed);

