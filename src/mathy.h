#pragma once
#include "main.h"

class Vector {
    public:
        double x;
        double y;
        double heading; //only used for specific scenarios TODO: find a better way of doing this
	    double magnitude();
	    Vector* rotate(double angle);
        double distance(Vector vector);
        double angle(); //returns radians
};

template<typename T>
T mathy_distance_between_points(T x1, T y1, T x2, T y2);

template<typename T>
T mathy_to_degrees(T angle);

template<typename T>
T mathy_to_radians(T angle);

double mathy_angle_wrap(double angle);

template<typename T>
T mathy_clamp(T value, T min, T max);

template<typename T>
T mathy_min(T x1, T x2);

template<typename T>
T mathy_max(T x1, T x2);

template<typename T>
T mathy_remap(T value, T from1, T to1, T from2, T to2);

//template implementations
//they are put in a header file due to templates not being "compiled"
#include "mathy.hxx"
