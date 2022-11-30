#ifndef LIDARDRIVE128S1_H
#define LIDARDRIVE128S1_H
#include "lidar_drive_interface.h"

#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class LidarDrive128S1 : public LidarDriveInterface
{
public:
    //LidarDrive128S1();

    // 使用继承构造函数
    //using LidarDriveInterface::LidarDriveInterface;
    virtual void run() override;
    virtual void generatePclCould() override;
    virtual void init() override;

    void init_thread();

    void udp_init(int *sock_num, sockaddr_in *sockaddr, int server_port, const std::string ip);

    void UnInitialize(int *sock_num);

    void setParams(const float &xangle, const float &yangle, const float &zangle,
                   const float &basex, const float &basey, const float &basez);

    // bool get_lidar_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg);


    private:

    LidarDataLS128S1 lidardata;

    struct sockaddr_in myaddr;
    struct sockaddr_in myClientAddr;
    int my_sock;

    float XAngle,YAngle,ZAngle = 0;
    float Base_X,Base_Y,Base_Z = 0;

};

#endif // LIDARDRIVE128S1_H
