#ifndef TRACKUTILS_H
#define TRACKUTILS_H

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "geodetic_conv.hpp"

namespace tracking{
/*Object in meters*/
struct obj_m{
    float x         = 0;
    float y         = 0;
    int frame       = -1;
    int cl          = -1;
    int w           = 0;
    int h           = 0;
    float error     = 0.0;
    float latitude  = 0.0;
    float longitude = 0.0;

    obj_m(){}
    obj_m(const float x_, const float y_, const int frame_, const int cl_, const int width, const int height, const float error_, const float latitude_, const float longitude_) : x(x_), y(y_), frame(frame_), cl(cl_), w(width), h(height), error(error_), latitude(latitude_), longitude(longitude_){}
    void print();
};
}

std::vector<tracking::obj_m> readDataFromFile(const std::string filename);

#endif /*TRACKUTILS_H*/
