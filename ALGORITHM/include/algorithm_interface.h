#ifndef ALGORITHMINTERFACE_H
#define ALGORITHMINTERFACE_H

#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "paramsevent.hpp"
#include "lidar_drive_interface.h"
#include "GroundMaterialDetection.h"
#include "ShipDetection.h"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

struct ProcessParamsOut
{
    AutomatedPort::ShipParamOut shipParamOut{};
    AutomatedPort::GroundParamOut groundParamOut{};
};

struct ProcessParamsIn
{
    AutomatedPort::ShipParamIn shipParamIn{};
    AutomatedPort::GroundParamIn groundParamIn{};
};

//template <class T>
class AlgorithmInterface
{
public:

    virtual void init() = 0;
    virtual void run() = 0;

    void globalParamsCallback(TotalParams msg,bool save_flag);
    bool get_lidar_data(ProcessParamsOut &msg);
    void initLidarDrive(const std::shared_ptr<LidarDriveInterface> msg);
    //bool get_lidar_data(T msg);

protected:
    std::mutex m_lock;
    std::queue<ProcessParamsOut> recv_lidar_queue_;
    //std::queue<T> recv_lidar_queue_;
    std::shared_ptr<LidarDriveInterface> lidarDriveInterface;
    ProcessParamsIn  process_params_in;
};

#endif // LALGORITHMINTERFACE_H
