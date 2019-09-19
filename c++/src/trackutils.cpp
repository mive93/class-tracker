#include "trackutils.h"

void Data::print()
{
    std::cout<<std::setprecision(10)<<"x: "<<x_<<"\ty: "<<y_<<"\tf: "<<frame_<<"\tc: "<<class_<<std::endl;
}


Data::Data()
{
    x_ = 0;
    y_ = 0;
    frame_ = -1;
    class_ = -1;
}

Data::Data(float x, float y, int frame, int classification)
{
    x_ = x;
    y_ = y;
    frame_ = frame;
    class_ = classification;
}


std::vector<Data> readDataFromFile(std::string filename)
{
    std::ifstream file;
    file.open(filename);

    Data d;
    long int timestamp;
    std::vector<Data> data;

    double east, north, up;

    geodetic_converter::GeodeticConverter gc;
    gc.initialiseReference(44.655540,10.934315, 0);

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
