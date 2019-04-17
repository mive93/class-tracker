#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct Data
{
    float x_;
    float y_;
    int frame_;

    void print();
};

std::vector<Data> readDataFromFile(std::string filename);


#endif /*UTILS_H*/