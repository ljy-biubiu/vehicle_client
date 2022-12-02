#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

using namespace std;

namespace planemethod {
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

    struct RTParams {
        double x;//配准平移距离，米
        double y;
        double z;
        double roll;// 配准旋转角度，度
        double pitch;
        double heading;
    };

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

    class PlaneMethod {
    public:
        PlaneMethod();

        ~PlaneMethod();

    public:
        // 根据点云计算平面模板方程
        void calcPlane(PointCloudTPtr inputCloud, double threshold, PlaneParams &planeParams);

        void calcNormalPlane(PointCloudTPtr inputCloud, double threshold, Eigen::Vector3f axis, PlaneParams &planeParams);

        // 计算2平面/法线的旋转矩阵
        void calcRotationBy2Plane(PlaneParams planeBefore, PlaneParams planeAfter, Eigen::Matrix4d &rotTrans, Eigen::Vector4d &projectionPoint);

        void printPlaneParams(PlaneParams planeParams);

    private:
        double normalLength;
    private:


    };

} // planemethod
