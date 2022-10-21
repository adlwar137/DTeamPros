#pragma once
#include "main.h"
#include "mathy.h"
#include "constants.h"

class OdometrySensor {
    public:
        OdometrySensor(pros::ADIEncoder* left_encoder,
                       pros::ADIEncoder* right_encoder,
                       pros::ADIEncoder* strafe_encoder);

        //one loop
        void updatePosition();
        //takes in a heading instead of relying on the tracking wheels
        void updatePosition(double heading);

        //while loop
        int32_t trackPosition(int32_t interval);
        //takes in a double pointer for a heading
        int32_t trackPosition(int32_t interval, double* heading);

        vector_t getPosition();
        double getHeading();
        vector3d_t getStatus();

        void reset();
    private:
        pros::ADIEncoder* left_encoder;
        pros::ADIEncoder* right_encoder;
        pros::ADIEncoder* strafe_encoder;

        vector3d_t pose;

        double getEncoderDistance(pros::ADIEncoder* encoder, double wheelDiameter);
};
/*
double odometry_get_encoder_distance(adi_encoder_t encoder, double wheel_diameter);

void odometry_track(void* tracking_params);
*/