#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <iostream>
#include <cstring>
#include <ctime>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#include "ShipDetection.h"
#include "GroundMaterialDetection.h"
#include "Util.h"
#include "algorithm_interface.h"

using namespace std;


class Algorithm : public AlgorithmInterface
{
public:
    Algorithm();

    virtual void init() override;
    virtual void run() override;

    void groundMaterialDetect();
    void showMaterial();
    void hatchDetectClouds();
    void show();
    static int getColorCloud(vector<PointCloudTPtr> cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorCloud,
                             int colorStart, int colorEnd);
    double timespecMs();

private:
    ProcessParamsOut params_out;
    bool updateFlag = false;
    bool running = true;

};

#endif // ALGORITHM_H
