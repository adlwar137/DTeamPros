#pragma once
#include "main.h"
#include "mathy.h"
#include <vector>

class GpsArray {
    public:
        GpsArray();
        void addGps(pros::Gps* sensor);

        Vector get_pose();
    private:
        std::vector<pros::Gps*> gpsSensors;
};