#ifndef MYPARAMS_H
#define MYPARAMS_H

#include "protocol.hpp"
#include <iostream>
#include <vector>

#include <pcl/io/ifs_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <opencv2/opencv.hpp>

#define PI_ 3.1415926

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;





struct PlaneParams {
    double A{};
    double B{};
    double C{};
    double D{};
    PointT centreOfGravity;
    PointT arrowPoint;
    pcl::ModelCoefficients::Ptr coefficients;
    PointCloudTPtr planeCloud;
    PointCloudTPtr nonPlaneCloud;
    PlaneParams(){
        coefficients.reset(new pcl::ModelCoefficients);
        planeCloud.reset(new PointCloudT);
        nonPlaneCloud.reset(new PointCloudT);
    }
};


struct RTParams {
    double x;//x方向 配准平移距离，米
    double y;
    double z;
    double roll;// 绕x轴 配准旋转角度，度，右手定则
    double pitch;
    double heading;

    RTParams() {
        x = 0;
        y = 0;
        z = 0;
        roll = 0;
        pitch = 0;
        heading = 0;
    }

    RTParams(double _roll, double _pitch, double _heading, double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
        roll = _roll;
        pitch = _pitch;
        heading = _heading;
    }
};


struct LineInfo {
    int index{};
    PointCloudTPtr cloud;
    PointCloudTPtr nonLineCloud;
    PointT centerPoint{};//中心点
    PointT direct{};//方向向量
    double w{};
    double l{};
    double h{};
    double heading{};
    PointT minP;//长度方向最近点
    PointT maxP;//长度方向最远点
    pcl::ModelCoefficients::Ptr coefficients;

    LineInfo() {
        cloud.reset(new PointCloudT);
        nonLineCloud.reset(new PointCloudT);
        coefficients.reset(new pcl::ModelCoefficients);
    }
};

struct RectangleInfo {
    std::vector<LineInfo> linesL;// 平行直线
    std::vector<LineInfo> linesW;// 平行直线
    std::vector<PointT> cornerPoints;
    double l;
    double w;
};

struct ObjectInfo {
    double x{};
    double y{};
    double z{};
    double w{};
    double l{};
    double h{};
    double heading{};
    bool usedFlag{};
    cv::RotatedRect rotatedRect;
    std::vector<cv::Point3f> cornerPoint;
    PointT minP;//长度方向最近点
    PointT maxP;//长度方向最远点
    PointT direct;//长度方向方向向量
    PointT directW;//长度方向方向向量
};

struct RectangleGrid {
    double l;
    double w;
    double heading;
    PointT centerPoint;
    PointT highestPoint;
    PointT lowestPoint;
    std::vector<PointT> cornerPoints;

    RectangleGrid() {
        l = 0;
        w = 0;
        heading = 0;
        centerPoint = {};
        highestPoint = {};
        lowestPoint = {};
    }
};


struct ShipParamIn {
    PointCloudTPtr cloudSrc;
    float boundRadius;//边界提取的半径
    float normalRadius;//法线提取的半径
    int gridNumX;//
    int gridNumY;
    RTParams rtParams;
    std::string roiFile;
    PointCloudTPtr cloudInited;
    double min_monitor_angle;
    double max_monitor_angle;

    ShipParamIn() {
        cloudSrc.reset(new PointCloudT);
        boundRadius = 2;
        normalRadius = 2;
        gridNumX = 0;
        gridNumY = 0;
        min_monitor_angle = 0;
        max_monitor_angle = 0;
        rtParams = {};
        roiFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/ShipROIPoints.txt";
        cloudInited.reset(new PointCloudT);
    }
};


struct GroundParamIn {
    PointCloudTPtr cloudSrc;
    int gridNumX;//
    int gridNumY;
    RTParams rtParams;
    std::string roiFile;
    std::string rectangleROIFile;//点顺序：左下角，右下角，右上角，左上角
    PointCloudTPtr cloudInited;
    double min_monitor_angle;
    double max_monitor_angle;

    GroundParamIn() {
        cloudSrc.reset(new PointCloudT);
        gridNumX = 0;
        gridNumY = 0;
        rtParams = {};
        min_monitor_angle = 0;
        max_monitor_angle = 0;
        roiFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/GroundROIPoints.txt";
        rectangleROIFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/GroundRectanglePoints.txt";
        cloudInited.reset(new PointCloudT);
    }
};

struct GroundParamOut {
    PointCloudTPtr cloudSrcRT;
    PlaneParams groundPlaneParams;
    PointCloudTPtr groundCloud;
    std::vector<PointT> materialHighestPoint;

    GroundParamOut() {
        cloudSrcRT.reset(new PointCloudT);
        groundCloud.reset(new PointCloudT);
        groundPlaneParams = {};
    }
};

struct ShipParamOut {
    PointCloudTPtr cloudSrcRT;//原始校正点云
    PlaneParams deckPlaneParams;// 甲板平面参数
    PointCloudTPtr deckCloud;// 甲板点云
    PointCloudTPtr deckBottomCloud;// 甲板底部点云
    PointCloudTPtr deckProjectionCloud;// 甲板投影点云
    PointCloudTPtr boundaryCloud;//边界点云
    std::vector<RectangleInfo> rectangleInfoVec;// 识别后的矩形信息
    std::vector<ObjectInfo> allClusters;// 所有聚类结果信息
    std::vector<PointT> bottomHighestPoint;// 底部最高点
    PointCloudTPtr polarCloud;// 极坐标点云
    std::vector<RectangleGrid> rectangleGridVec;// 每个矩形栅格信息

    ShipParamOut() {
        cloudSrcRT.reset(new PointCloudT);
        deckCloud.reset(new PointCloudT);
        deckBottomCloud.reset(new PointCloudT);
        deckProjectionCloud.reset(new PointCloudT);
        boundaryCloud.reset(new PointCloudT);
        polarCloud.reset(new PointCloudT);
        deckPlaneParams = {};
    }
};


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
    GroundParamIn groundParamIn;
    ShipParamIn shipParamIn;

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
