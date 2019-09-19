#ifndef TRACKUTILS_H
#define TRACKUTILS_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include "geodetic_conv.hpp"

struct Data
{
    float x_;
    float y_;
    int frame_;
    int class_;

    Data();
    Data(float x, float y, int frame, int classification);
    void print();
};

std::vector<Data> readDataFromFile(std::string filename);


#endif /*TRACKUTILS_H*/