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
    float x     = 0;
    float y     = 0;
    int frame   = -1;
    int cl      = -1;
    int w       = 0;
    int h       = 0;

    obj_m(){}
    obj_m(const float x_, const float y_, const int frame_, const int cl_, const int width, const int height) : x(x_), y(y_), frame(frame_), cl(cl_), w(width), h(height) {}
    void print();
};
}

std::vector<tracking::obj_m> readDataFromFile(const std::string filename);

#endif /*TRACKUTILS_H*/