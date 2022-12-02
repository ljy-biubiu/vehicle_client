#pragma once

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

using namespace std;

namespace lshapedfit {
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

#define MAX_FLOAT 3.40282347e+38F

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

    class LShapedFIT {
    public:
        LShapedFIT();

        ~LShapedFIT();

        void fitBox(PointCloudTPtr _cloud, ObjectInfo &_cluster);

        std::vector<cv::Point3f> getRectVertex();

        const ObjectInfo &getCluster() const;

        void setCluster(const ObjectInfo &_cluster);

    private:
        // For Each ObjectInfo.
        cv::RotatedRect FitBox(std::vector<cv::Point2f> *pointcloud_ptr);

        // Different Criterion For ObjectInfo BBox.
        double calc_area_criterion(const cv::Mat &c1, const cv::Mat &c2);

        double calc_nearest_criterion(const cv::Mat &c1, const cv::Mat &c2);

        double calc_variances_criterion(const cv::Mat &c1, const cv::Mat &c2);

        double calc_var(const std::vector<double> &v);

        void calc_cross_point(const double a0, const double a1, const double b0, const double b1, const double c0,
                              const double c1, double &x, double &y);

        cv::RotatedRect calc_rect_contour();

        double dist2Points(const cv::Point3f &p1, const cv::Point3f &p2);

    public:
        enum Criterion {
            AREA,
            NEAREST,
            VARIANCE
        };

    private:
        double min_dist_of_nearest_crit_;
        double dtheta_deg_for_search_;

        Criterion criterion_;

        std::vector<double> a_;
        std::vector<double> b_;
        std::vector<double> c_;

        cv::Point3f minP;
        cv::Point3f maxP;
        std::vector<cv::Point3f> vertex_pts_;
        cv::Point2f hot_pt_;

        ObjectInfo objectInfo;

    };  // class LShapedFIT
}

