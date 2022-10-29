#include "PIDController.h"

PIDController::PIDController(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

double PIDController::calculate(double error) {
    this->integral = this->integral + error;
    this->derivative = error - this->prevError;
    this->prevError = error;

    return (this->Kp * error +
            this->Ki * this->integral +
            this->Kd * this->derivative);
}