#pragma once
#include "main.h"
#include "mathy.h"
#include "constants.h"

using namespace pros::c;

typedef struct tracking_params {
    adi_encoder_t left_encoder;
    adi_encoder_t right_encoder;
    adi_encoder_t strafe_encoder;
    vector3d *pose;
} tracking_params_t;

double odometry_get_encoder_distance(adi_encoder_t encoder, double wheel_diameter);

void odometry_track(void* tracking_params);