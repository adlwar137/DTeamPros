#pragma once
#include "main.h"
#include "mathy.h"

using namespace pros::c;

class PIDController {
    public:
        PIDController(double Kp, double Ki, double Kd);

        PIDController(double Kp, double Ki, double Kd, double integralMax);

        double calculate(double error);

        int32_t setIntegralMax(double max);
    private:
        double Kp;
        double Ki;
        double Kd;
        double prevError;
        double integral;
        double derivative;
        double integralMax;
        bool isIntegralMax;
};