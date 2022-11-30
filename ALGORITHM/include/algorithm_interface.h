#ifndef ALGORITHMINTERFACE_H
#define ALGORITHMINTERFACE_H

#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "paramsevent.hpp"
#include "lidar_drive_interface.h"

//template <class T>
class AlgorithmInterface
{
public:

    virtual void init() = 0;
    virtual void run() = 0;

    void globalParamsCallback(TotalParams msg,bool save_flag);
    bool get_lidar_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg);
    void initLidarDrive(const std::shared_ptr<LidarDriveInterface> msg);
    //bool get_lidar_data(T msg);

protected:
    std::mutex m_lock;
    std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> recv_lidar_queue_;
    //std::queue<T> recv_lidar_queue_;
    std::shared_ptr<LidarDriveInterface> lidarDriveInterface;
};

#endif // LALGORITHMINTERFACE_H
