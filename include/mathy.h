#pragma once
#include "main.h"


typedef struct vector {
    int32_t x;
    int32_t y;
} vector_t;

//mostly generic but used as x, y, and heading
typedef struct vector3d {
    double x; //inches
    double y; //inches
    double w; //radians
} vector3d;

vector_t mathy_rotate_vector(vector_t vec, double angle); 

vector3d mathy_rotate_vector3d(vector3d vec, double angle);

double mathy_distance_between_points(vector vec1, vector vec2);

double mathy_distance_between_points(double x1, double y1, double x2, double y2);

double mathy_to_degrees(double angle);

double mathy_to_radians(double angle);

double mathy_angle_wrap(double angle);

double mathy_clamp(double value, double min, double max);

int32_t mathy_remap(int32_t value, int32_t from1, int32_t to1, int32_t from2, int32_t to2);