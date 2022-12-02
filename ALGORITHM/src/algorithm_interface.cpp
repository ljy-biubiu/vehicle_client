#include "algorithm_interface.h"


bool AlgorithmInterface::get_lidar_data( ProcessParamsOut &msg )
{
    if (recv_lidar_queue_.size() > 0)
    {
        std::lock_guard<std::mutex>(this->m_lock);
        msg = std::move(this->recv_lidar_queue_.front());
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
    process_params_in.shipParamIn.gridNumX = msg.shipParamIn.gridNumX;
    process_params_in.shipParamIn.gridNumY = msg.shipParamIn.gridNumY;
    process_params_in.shipParamIn.boundRadius = msg.shipParamIn.boundRadius;
    process_params_in.shipParamIn.normalRadius = msg.shipParamIn.normalRadius;
    process_params_in.shipParamIn.max_monitor_angle = msg.shipParamIn.max_monitor_angle;
    process_params_in.shipParamIn.min_monitor_angle = msg.shipParamIn.min_monitor_angle;

    process_params_in.groundParamIn.gridNumX = msg.groundParamIn.gridNumX;
    process_params_in.groundParamIn.gridNumY = msg.groundParamIn.gridNumY;
    process_params_in.groundParamIn.max_monitor_angle = msg.groundParamIn.max_monitor_angle;
    process_params_in.groundParamIn.min_monitor_angle = msg.groundParamIn.min_monitor_angle;

//    std::cout<<process_params_in.shipParamIn.gridNumX<<std::endl;
//    std::cout<<process_params_in.shipParamIn.gridNumY<<std::endl;
//    std::cout<<process_params_in.shipParamIn.boundRadius<<std::endl;
//    std::cout<<process_params_in.shipParamIn.normalRadius<<std::endl;
//    std::cout<<process_params_in.shipParamIn.max_monitor_angle<<std::endl;
//    std::cout<<process_params_in.shipParamIn.min_monitor_angle<<std::endl;

//    std::cout<<process_params_in.groundParamIn.gridNumX<<std::endl;
//    std::cout<<process_params_in.groundParamIn.gridNumY<<std::endl;
//    std::cout<<process_params_in.groundParamIn.max_monitor_angle<<std::endl;
//    std::cout<<process_params_in.groundParamIn.min_monitor_angle<<std::endl;
}

void AlgorithmInterface::initLidarDrive(const std::shared_ptr<LidarDriveInterface> msg)
{

}
