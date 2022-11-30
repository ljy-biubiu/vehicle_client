#include "communication_interface.h"

bool ComunicateInterface::get_receive_data( ReadData &msg)
{
    if (recv_data_queue_.size() > 0)
    {
        std::lock_guard<std::mutex>(this->m_lock);
        msg = std::move(this->recv_data_queue_.front());
        this->recv_data_queue_.pop();
        return true;
    }
    else
    {
        return false;
    }
}


void ComunicateInterface::globalParamsCallback(TotalParams msg,bool save_flag)
{
    this->my_port = msg.lowerMachineBase.port;
    this->my_ip = msg.lowerMachineBase.ip;
}

void ComunicateInterface::initParamsEvent(std::shared_ptr<ParamsEvent<ObserverFunc>> params_event_)
{
    params_event = params_event_;

    std::lock_guard<std::mutex>(params_event->getMylock());

    params_event->getTotalParams().lidarBase.DataAddress = "dasdasd";
    params_event->Notify(params_event->getTotalParams(),false);
}
