#include "mathy.h"

vector_t mathy_rotate_vector(vector_t vec, double angle) {
    vec.x = vec.x*cos(angle) - vec.y*sin(angle);
    vec.y = vec.x*sin(angle) + vec.y*cos(angle);
    return vec;
}

vector3d mathy_rotate_vector3d(vector3d vec, double angle) {
  vec.x = vec.x*cos(angle) - vec.y*sin(angle);
  vec.y = vec.x*sin(angle) + vec.y*cos(angle);
  return vec;
}

double mathy_distance_between_points(vector firstVec, vector secondVec) {
  return sqrt(pow(firstVec.x - secondVec.x, 2) + pow(firstVec.y - secondVec.y, 2));
}

double mathy_distance_between_points(double firstX, double secondX, double firstY, double secondY) {
  return sqrt(pow(firstX - secondX, 2) + pow(firstY - secondY, 2));
}

double mathy_to_degrees(double angle) {
  return (angle/M_PI) * 180;
}

double mathy_to_radians(double angle) {
  return (angle/180) * M_PI;
}

double mathy_angle_wrap(double angle) {
    return atan2(sin(angle), cos(angle));
}

double mathy_clamp(double value, double min, double max) {
  if(value > max) {
    value = max;
  }
  if(value < min) {
    value = min;
  }
  return value;
}

int32_t mathy_remap(int32_t value, int32_t from1, int32_t to1, int32_t from2, int32_t to2) {
  return (int)((double)(value - from1) / (to1 - from1) * (to2 - from2) + from2);
}