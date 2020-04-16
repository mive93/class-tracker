#ifndef TRACKUTILS_H
#define TRACKUTILS_H

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "geodetic_conv.hpp"

struct Data
{
    float x_    = 0;
    float y_    = 0;
    int frame_  = -1;
    int class_  = -1;

    Data(){}
    Data(const float x, const float y, const int frame, const int classification) : x_(x), y_(y), frame_(frame), class_(classification) {}
    void print();
};

std::vector<Data> readDataFromFile(const std::string filename);

#endif /*TRACKUTILS_H*/