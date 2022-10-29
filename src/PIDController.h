#pragma once
#include "main.h"

using namespace pros::c;

class PIDController {
    public:
        PIDController(double Kp, double Ki, double Kd);

        double calculate(double error);
    private:
        double Kp;
        double Ki;
        double Kd;
        double prevError;
        double integral;
        double derivative;
};