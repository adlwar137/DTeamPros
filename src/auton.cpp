#include "auton.h"

void auton_roller_swap() {
    base_move_velocity(base, 0, 200, 0);
    motor_move(INTAKE, -127);
    delay(500);
    motor_brake(INTAKE);
    base_brake(base);
}