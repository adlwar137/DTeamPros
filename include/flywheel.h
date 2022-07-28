#include "main.h"

typedef struct flywheel {
    uint8_t motorA;
    uint8_t motorB;
} flywheel;

int32_t flywheel_spin(flywheel Flywheel, const int8_t speed);

int32_t flywheel_set_gearing(flywheel Flywheel, const pros::motor_gearset_e gearset);

int32_t flywheel_set_reversed(flywheel Flywheel, const bool reversed);