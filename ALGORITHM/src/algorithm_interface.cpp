#include "algorithm_interface.h"


bool AlgorithmInterface::get_lidar_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg)
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

void AlgorithmInterface::globalParamsCallback(TotalParams msg,bool save_flag)
{

}

void AlgorithmInterface::initLidarDrive(const std::shared_ptr<LidarDriveInterface> msg)
{

}
