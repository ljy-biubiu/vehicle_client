#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>


void read_xyz(std::string fname, csf::PointCloud &pointcloud) {
    std::ifstream fin(fname.c_str(), std::ios::in);
    char line[500];
    std::string x, y, z;

    while (fin.getline(line, sizeof(line))) {
        std::stringstream words(line);

        words >> x;
        words >> y;
        words >> z;

        csf::Point point;
        point.x = atof(x.c_str());
        point.y = -atof(z.c_str());
        point.z = atof(y.c_str());

        pointcloud.push_back(point);
    }
}
