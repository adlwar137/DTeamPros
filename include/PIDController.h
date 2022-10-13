#pragma once
#include "main.h"

using namespace pros::c;

typedef struct PIDController {
    double Kp;
    double Ki;
    double Kd;
    double error;
    double prevError;
    double integral;
    double derivative;
} PIDController_t;

PIDController_t PIDController_create(double Kp, double Ki, double Kd);

double PIDController_calculate(PIDController_t pidController, double error);