#ifndef MYPARAMS_H
#define MYPARAMS_H

#include "protocol.hpp"
#include<iostream>


struct LidarPreset
{
    double paintarea_Radius{0};

    double ZAngle{0};
    double XAngle{0};
    double YAngle{0};

    double Base_X{0};
    double Base_Y{0};
    double Base_Z{0};
};

struct LidarCluster
{
    double ClusterTolerance{0};
    double MinClusterSize{0};
    double MaxClusterSize{0};
    double Resolution{0};
    double Difference_threshold{0};
    double Kvalue{0};
    double EuclideanDistance{0};
};

struct LidarBase
{
    int lidarPort{0};
    std::string LidarAddress;

    int ServerPort{0};
    std::string ServerAddress;

    std::string DataAddress;
    int DeviceId{0};

    int LidarModel{0};

};

struct LidarCameraBase
{
    double Pos_X{0};
    double Pos_Y{0};
    double Pos_Z{50.0};

    double View_X{0};
    double View_Y{0};
    double View_Z{0};

    double Up_X{0};
    double Up_Y{0};
    double Up_Z{0};
};

struct CameraBase
{
    std::string ip{"127.0.0.1"};
    std::string name{"admin123"};
    std::string code{"ls123456"};
};

struct LowerMachineBase
{
    std::string ip{"127.0.0.1"};
    int port{2368};
};


struct TotalParams
{
    ComunicationData protocol_data;
    LidarPreset lidarPreset;
    LidarCluster lidarCluster;
    LidarBase lidarBase;
    LidarCameraBase lidarCameraBase;
    CameraBase cameraBase;
    LowerMachineBase lowerMachineBase;

    void printData()
    {
        /////////////////////////////
        //PLC协议设置内容
        std::cout<<this->protocol_data.read_data.angle<<std::endl;
        std::cout<<this->protocol_data.read_data.amplitude_position<<std::endl;
        std::cout<<this->protocol_data.read_data.height<<std::endl;
        std::cout<<this->protocol_data.read_data.safe_distance<<std::endl;
        std::cout<<this->protocol_data.read_data.resolution<<std::endl;
        std::cout<<this->protocol_data.read_data.build_map_signal<<std::endl;
        std::cout<<this->protocol_data.read_data.comnunication_flag<<std::endl;
        std::cout<<this->protocol_data.read_data.device_status<<std::endl;
        std::cout<<this->protocol_data.read_data.material_area_x<<std::endl;
        std::cout<<this->protocol_data.read_data.material_area_y<<std::endl;
        std::cout<<this->protocol_data.read_data.meterial_area_length<<std::endl;
        std::cout<<this->protocol_data.read_data.meterial_area_width<<std::endl;
        std::cout<<this->protocol_data.read_data.technology<<std::endl;

        //
        std::cout<<this->lidarPreset.paintarea_Radius<<std::endl;
        std::cout<<this->lidarPreset.XAngle <<std::endl;
        std::cout<<this->lidarPreset.YAngle <<std::endl;
        std::cout<<this->lidarPreset.ZAngle <<std::endl;
        std::cout<<this->lidarPreset.Base_X <<std::endl;
        std::cout<<this->lidarPreset.Base_Y <<std::endl;
        std::cout<<this->lidarPreset.Base_Z <<std::endl;

        //
        std::cout<<this->lidarCluster.ClusterTolerance <<std::endl;
        std::cout<<this->lidarCluster.MinClusterSize<<std::endl;
        std::cout<<this->lidarCluster.MaxClusterSize <<std::endl;
        std::cout<<this->lidarCluster.Resolution <<std::endl;
        std::cout<<this->lidarCluster.Difference_threshold <<std::endl;
        std::cout<<this->lidarCluster.Kvalue <<std::endl;
        std::cout<<this->lidarCluster.EuclideanDistance <<std::endl;


        //
        std::cout<<this->lidarBase.lidarPort <<std::endl;
        std::cout<<this->lidarBase.LidarAddress <<std::endl;
        std::cout<<this->lidarBase.ServerPort <<std::endl;
        std::cout<<this->lidarBase.ServerAddress <<std::endl;
        std::cout<<this->lidarBase.DataAddress <<std::endl;
        std::cout<<this->lidarBase.DeviceId <<std::endl;
        std::cout<<this->lidarBase.LidarModel <<std::endl;


        //
        std::cout<<this->lidarCameraBase.Pos_X <<std::endl;
        std::cout<<this->lidarCameraBase.Pos_Y <<std::endl;
        std::cout<<this->lidarCameraBase.Pos_Z <<std::endl;
        std::cout<<this->lidarCameraBase.View_X <<std::endl;
        std::cout<<this->lidarCameraBase.View_Y <<std::endl;
        std::cout<<this->lidarCameraBase.View_Z <<std::endl;
        std::cout<<this->lidarCameraBase.Up_X <<std::endl;
        std::cout<<this->lidarCameraBase.Up_Y <<std::endl;
        std::cout<<this->lidarCameraBase.Up_Z <<std::endl;

        std::cout<<this->cameraBase.ip <<std::endl;
        std::cout<<this->cameraBase.code <<std::endl;
        std::cout<<this->cameraBase.name <<std::endl;
        std::cout<<this->lowerMachineBase.ip <<std::endl;
        std::cout<<this->lowerMachineBase.port <<std::endl;
    }
};

#endif // MYPARAMS_H
