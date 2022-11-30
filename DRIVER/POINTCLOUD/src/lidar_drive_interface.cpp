#include "lidar_drive_interface.h"


bool LidarDriveInterface::get_lidar_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg)
{
    if (recv_lidar_queue_.size() > 0)
    {
        std::lock_guard<std::mutex>(this->m_lock);
        *msg = std::move(*this->recv_lidar_queue_.front());
        this->recv_lidar_queue_.pop();
        return true;
    }
    else
    {
        return false;
    }
}

void LidarDriveInterface::globalParamsCallback(TotalParams msg,bool save_flag)
{
    this->my_ip  = msg.lidarBase.LidarAddress;
    this->my_port = msg.lidarBase.lidarPort;
}

void LidarDriveInterface::setParams(const int port, const std::string ip)
{
    this->my_ip  = ip;
    this->my_port = port;
}
