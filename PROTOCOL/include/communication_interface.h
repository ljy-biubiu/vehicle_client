#ifndef COMMUNICATION_INTERFACE_H
#define COMMUNICATION_INTERFACE_H

#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <array>
#include "protocol.hpp"
#include "paramsevent.hpp"




class ComunicateInterface
{
public:
    virtual void init() = 0;
    virtual void run() = 0;
    virtual bool send_data(WriteData &msg) = 0;

    bool get_receive_data(ReadData &msg);
    void globalParamsCallback(TotalParams msg,bool save_flag);
    void initParamsEvent(std::shared_ptr<ParamsEvent<ObserverFunc>> params_event_);
    
protected:
    std::mutex m_lock;
    std::queue<ReadData> recv_data_queue_;

    int my_port;
    std::string my_ip;

private:
    std::shared_ptr<ParamsEvent<ObserverFunc>> params_event;


};

#endif // COMMUNICATION_INTERFACE_H
