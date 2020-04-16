#include "Data.h"

void Data::print()
{
    std::cout << std::setprecision(10) << "x: " << x_ << "\ty: " << y_ << "\tf: " << frame_ << "\tc: " << class_ << std::endl;
}

std::vector<Data> readDataFromFile(const std::string filename)
{
    std::ifstream file;
    file.open(filename);

    Data d;
    long int timestamp;
    std::vector<Data> data;

    double east, north, up;

    geodetic_converter::GeodeticConverter gc;
    gc.initialiseReference(44.655540, 10.934315, 0); //set the origin (centre)

    if (file.is_open())
    {
        std::string line;
        while (getline(file, line))
        {
            std::istringstream iss(line);
            iss >> d.frame_ >> timestamp >> d.x_ >> d.y_;

            //conversion from lat lon to distance(m) from centre
            gc.geodetic2Enu(d.x_, d.y_, 0, &east, &north, &up);

            d.x_ = east;
            d.y_ = north;

            data.push_back(d);
        }
    }

    file.close();
    return data;
}
