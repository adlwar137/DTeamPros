#include "config.h"
#include "main.h"

pros::Motor frontLeft = pros::Motor(4, false);
pros::Motor frontRight = pros::Motor(1, true);
pros::Motor backLeft = pros::Motor(3, false);
pros::Motor backRight = pros::Motor(2, true);

pros::Motor flywheelLeft = pros::Motor(5, true);
pros::Motor flywheelRight = pros::Motor(7, false);

Chassis base = Chassis(&frontLeft, &frontRight, &backLeft, &backRight, pros::motor_gearset_e_t::E_MOTOR_GEARSET_18);

Flywheel discShooter = Flywheel(&flywheelLeft, &flywheelRight, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06);