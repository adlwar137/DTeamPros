#include "main.h"

typedef struct chassis {
	uint8_t frontLeftMotor;	
	uint8_t frontRightMotor;
	uint8_t backLeftMotor;
	uint8_t backRightMotor;
} chassis_t;

int32_t base_move(chassis_t chassis, const int32_t x, const int32_t y);

int32_t base_turn(chassis_t chassis, const int32_t x);

int32_t base_move_velocity(chassis_t chassis, const int32_t x, const int32_t y);

int32_t base_turn_velocity(chassis_t chassis, const int32_t x);

int32_t base_set_gearing(chassis_t chassis, const pros::motor_gearset_e gearset);

int32_t base_set_reversed(chassis_t chassis, const bool reversed);

