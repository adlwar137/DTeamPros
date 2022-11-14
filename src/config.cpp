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

pros::Vision rollerSensor = pros::Vision(18);

pros::vision_signature RED_ = rollerSensor.signature_from_utility(0, 8135, 12379, 10257, -1301, -257, -779, 2.5, 0);

pros::ADIDigitalOut piston = pros::ADIDigitalOut(7, true);
pros::ADIDigitalOut rollerLED = pros::ADIDigitalOut({17, 3}, LOW);

pros::ADIUltrasonic rangeFinder = pros::ADIUltrasonic({17, 1, 2});

Chassis base = Chassis(&frontLeft, &frontRight, &backLeft, &backRight, pros::E_MOTOR_GEARSET_18);

Flywheel flywheel = Flywheel(&flywheelLeft, &flywheelRight, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06);

OdometrySensor odometry = OdometrySensor(&left_encoder, &right_encoder, &strafe_encoder);