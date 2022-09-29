#include "main.h"

using namespace pros::c;

typedef struct pose {
    int32_t x; //inches
    int32_t y; //inches
    int32_t w; //radians
} pose_t;

typedef struct tracking_params {
    adi_encoder_t left_encoder;
    adi_encoder_t right_encoder;
    adi_encoder_t strafe_encoder;
    pose_t pose;
} tracking_params_t;

#define TRACKING_WHEEL_DIAMETER 2.75
#define LEFT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 3.75
#define RIGHT_TRACKING_WHEEL_DISTANCE_FROM_CENTER 3.5
#define STRAFE_TRACKING_WHEEL_DISTANCE_FROM_CENTER 4.75

double odometry_get_encoder_distance(adi_encoder_t encoder, double wheel_diameter);

void odometry_track(void* tracking_params);