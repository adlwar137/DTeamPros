#pragma once
#include "main.h"
#include "mathy.h"

using namespace pros::c;

typedef struct tracking_params {
    adi_encoder_t left_encoder;
    adi_encoder_t right_encoder;
    adi_encoder_t strafe_encoder;
    vector3d *pose;
} tracking_params_t;

#define TRACKING_WHEEL_DIAMETER 2.75
#define LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 3.75
#define RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 3.5
#define STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4.75
#define POWERED_WHEEL_DISTANCE_FROM_CENTER 8.25

double odometry_get_encoder_distance(adi_encoder_t encoder, double wheel_diameter);

void odometry_track(void* tracking_params);