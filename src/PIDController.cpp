#include "PIDController.h"

PIDController::PIDController(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->integralMax = 0;
    this->integral = 0;
    this->derivative = 0;
    this->prevError = 0;
}

PIDController::PIDController(double Kp, double Ki, double Kd, double integralMax) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->integralMax = integralMax;
    this->integral = 0;
    this->derivative = 0;
    this->prevError = 0;
}

double PIDController::calculate(double error) {
    this->integral = this->integral + error;
    this->derivative = error - this->prevError;
    this->prevError = error;

    if(this->integralMax != 0) {
        this->integral = mathy_clamp<double>(this->integral, -integralMax, integralMax);
    }

    return ((this->Kp * error) +
            (this->Ki * this->integral) +
            (this->Kd * this->derivative));
}