#include "mathy.h"

bool mathy_within(double x, double y, double tolerance) {
  return fabs(y - x) < tolerance;
}

double Vector::magnitude() {
  return hypot(this->x, this->y);
}

Vector* Vector::rotate(double angle) {
  this->x = this->x*cos(angle) - this->y*sin(angle);
  this->x = this->x*sin(angle) + this->y*cos(angle);
  return this;
}

double Vector::distance(Vector vector) {
  return sqrt(pow(this->x - vector.x, 2) + pow(this->y - vector.y, 2));
}

double Vector::angle() {
  return atan2(this->y, this->x);
}

double mathy_angle_wrap(double angle) {
    return atan2(sin(angle), cos(angle));
}