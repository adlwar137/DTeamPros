#include "config.h"
#include "main.h"

pros::Motor frontLeft = pros::Motor(4, false);
pros::Motor frontRight = pros::Motor(1, true);
pros::Motor backLeft = pros::Motor(3, false);
pros::Motor backRight = pros::Motor(2, true);

pros::Motor flywheelLeft = pros::Motor(5, true);
pros::Motor flywheelRight = pros::Motor(7, false);

pros::ADIEncoder left_encoder = pros::ADIEncoder(3, 4, false);
pros::ADIEncoder right_encoder = pros::ADIEncoder(1, 2, false);
pros::ADIEncoder strafe_encoder = pros::ADIEncoder(5, 6, false);

pros::Imu inertial = pros::Imu(13);

Chassis base = Chassis(&frontLeft, &frontRight, &backLeft, &backRight, pros::E_MOTOR_GEARSET_18);

Flywheel discShooter = Flywheel(&flywheelLeft, &flywheelRight, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06);

OdometrySensor odometry = OdometrySensor(&left_encoder, &right_encoder, &strafe_encoder);