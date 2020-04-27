#include "obj.h"

namespace tracking{
void obj_m::print(){
    std::cout << std::setprecision(10) << "x: " << x << "\ty: " << y << "\tf: " << frame << "\tc: " << cl << std::endl;
}
}

std::vector<tracking::obj_m> readDataFromFile(const std::string filename){
    std::ifstream file;
    file.open(filename);

    tracking::obj_m d;
    long int timestamp;
    std::vector<tracking::obj_m> data;

    double east, north, up;

    geodetic_converter::GeodeticConverter gc;
    gc.initialiseReference(44.655540, 10.934315, 0); //set the origin (centre)

    if (file.is_open()){
        std::string line;
        while (getline(file, line)){
            std::istringstream iss(line);
            iss >> d.frame >> timestamp >> d.x >> d.y;

            //conversion from lat lon to distance(m) from centre
            gc.geodetic2Enu(d.x, d.y, 0, &east, &north, &up);

            d.x = east;
            d.y = north;

            data.push_back(d);
        }
    }

    file.close();
    return data;
}
