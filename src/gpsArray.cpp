#include "gpsArray.h"

GpsArray::GpsArray() {

}

void GpsArray::addGps(pros::Gps* sensor) {
    this->gpsSensors.push_back(sensor);
}

//just average gps values to get a pose
//TODO
//will come up with smarter solution later
Vector GpsArray::get_pose() {
    Vector poseSum;
    //sum all the sensors values
    for(int i = 0; i < this->gpsSensors.size(); i++) {
        poseSum.x += this->gpsSensors.at(i)->get_status().x;
        poseSum.y += this->gpsSensors.at(i)->get_status().y;
        poseSum.heading += this->gpsSensors.at(i)->get_rotation();
    }
    //divide by size to average them
    poseSum.x /= this->gpsSensors.size();
    poseSum.y /= this->gpsSensors.size();
    poseSum.heading /= this->gpsSensors.size();
    //convert to radians
    poseSum.heading = mathy_to_radians<double>(poseSum.heading);
    //wrap angle
    poseSum.heading = mathy_angle_wrap(poseSum.heading);

    return poseSum;
}