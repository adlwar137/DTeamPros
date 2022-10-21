#include "gpsArray.h"

GpsArray::GpsArray() {

}

void GpsArray::addGps(pros::Gps* sensor) {
    this->gpsSensors.push_back(sensor);
}

//just average gps values to get a pose
//TODO
//will come up with smarter solution later
vector3d_t GpsArray::get_pose() {
    vector3d_t poseSum;
    //sum all the sensors values
    for(int i = 0; i < this->gpsSensors.size(); i++) {
        poseSum.x += this->gpsSensors.at(i)->get_status().x;
        poseSum.y += this->gpsSensors.at(i)->get_status().y;
        poseSum.w += this->gpsSensors.at(i)->get_rotation();
    }
    //divide by size to average them
    poseSum.x /= this->gpsSensors.size();
    poseSum.y /= this->gpsSensors.size();
    poseSum.w /= this->gpsSensors.size();
    //convert to radians
    poseSum.w = mathy_to_radians(poseSum.w);
    //wrap angle
    poseSum.w = mathy_angle_wrap(poseSum.w);

    return poseSum;
}