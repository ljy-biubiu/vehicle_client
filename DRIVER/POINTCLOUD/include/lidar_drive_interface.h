#ifndef LIDARDRIVEINTERFACE_H
#define LIDARDRIVEINTERFACE_H

#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include <pcl/io/ifs_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <common.h>

#include "paramsevent.hpp"

#define PI_ 3.1415926

class LidarDriveInterface
{
public:

    virtual void init() = 0;
    virtual void run() = 0;
    virtual void generatePclCould() = 0;

    void globalParamsCallback(TotalParams msg,bool save_flag);
    bool get_lidar_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg);
    void setParams(const int port, const std::string ip);

protected:
    int my_port;
    std::string my_ip;
    std::mutex m_lock;
    std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> recv_lidar_queue_;
};

#endif // LIDARDRIVEINTERFACE_H
