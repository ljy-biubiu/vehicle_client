#ifndef XYZ_READER_H_
#define XYZ_READER_H_

#include <string>
#include <vector>
#include "point_cloud.h"

void read_xyz(std::string fname, csf::PointCloud &pointcloud);


#endif // ifndef XYZ_READER_H_
