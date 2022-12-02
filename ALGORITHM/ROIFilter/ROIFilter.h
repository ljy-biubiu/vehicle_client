#pragma once

#include <iostream>
#include <cstring>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "hdmapRoiFilter.h"
#include "../UtilMethod/Util.h"

using namespace std;

namespace roifilter {
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

    class ROIFilter {
    public:
        ROIFilter();

        ~ROIFilter();

    public:
        void roiFilter(PointCloudTPtr cloud_in, std::string pointsPath, double cell_size_, PointCloudTPtr &cloud_out);

        void roiFilter(PointCloudTPtr cloud_in, std::vector<cv::Point2d> polygon, double cell_size_, PointCloudTPtr &cloud_out);

        void readRegionPoints(std::string pointsPath, std::vector<cv::Point2d> &polygon);

    private:

    private:


    };

} // roifilter
