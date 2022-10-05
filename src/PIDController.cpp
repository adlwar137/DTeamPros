#include "PIDController.h"

PIDController_t PIDController_create(double Kp, double Ki, double Kd) {
    PIDController_t temp;
    temp.Kp = Kp;
    temp.Ki = Ki;
    temp.Kd = Kd;
    return temp;
}

double PIDController_calculate(PIDController_t pidController, double error) {
    pidController.integral = pidController.integral + error;
    pidController.derivative = error - pidController.prevError;
    pidController.prevError = error;

    return (pidController.Kp * error +
            pidController.Ki * pidController.integral +
            pidController.Kd * pidController.derivative);
}