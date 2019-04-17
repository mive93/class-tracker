#include "utils.h"

void Data::print()
{
    std::cout<<x_<<" "<<y_<<" "<<frame_<<std::endl;
}

std::vector<Data> readDataFromFile(std::string filename)
{
    std::ifstream file;
    file.open(filename);

    Data d;
    long int timestamp;
    std::vector<Data> data;

    if (file.is_open())
    {
        std::string line;
        while (getline(file, line))
        {
            std::istringstream iss(line);
            iss >> d.frame_ >> timestamp >> d.x_ >> d.y_;
            data.push_back(d);
        }
    }

    file.close();
    return data;
}
