#include "flywheel.h"

using namespace pros::c;

int32_t flywheel_spin(flywheel Flywheel, const int8_t speed) {
    motor_move(Flywheel.motorA, speed);
    motor_move(Flywheel.motorB, speed);
    return 1;
}

int32_t flywheel_set_gearing(flywheel Flywheel, const pros::motor_gearset_e gearset) {
    motor_set_gearing(Flywheel.motorA, gearset);
    motor_set_gearing(Flywheel.motorB, gearset);
    return 1;
}

int32_t flywheel_set_reversed(flywheel Flywheel, const bool reversed) {
    motor_set_reversed(Flywheel.motorA, reversed);
    motor_set_reversed(Flywheel.motorB, reversed);
    return 1;
}