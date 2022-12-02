#pragma once

#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "ROIFilter.h"

using namespace std;

namespace AutomatedPort {
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

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
        vector<LineInfo> linesL;// 平行直线
        vector<LineInfo> linesW;// 平行直线
        vector<PointT> cornerPoints;
        double l;
        double w;
    };

    struct RectangleGrid {
        double l;
        double w;
        double heading;
        PointT centerPoint;
        PointT highestPoint;
        PointT lowestPoint;
        vector<PointT> cornerPoints;

        RectangleGrid() {
            l = 0;
            w = 0;
            heading = 0;
            centerPoint = {};
            highestPoint = {};
            lowestPoint = {};
        }
    };

    struct GridCell {
        int ix;
        int jy;
        int kz;
        unsigned long long id;

        GridCell() {
            ix = 1;
            jy = 1;
            kz = 1;
            id = 1;
        }

        GridCell(int _ix, int _jy, int _kz, unsigned long long _id) {
            ix = _ix;
            jy = _jy;
            kz = _kz;
            id = _id;
        }

        bool operator<(const GridCell &gridCell) const {
            if (id < gridCell.id) {
                return true;
            }
            return false;
        }
    };

    class BaseDetection {
    public:
        BaseDetection();

        ~BaseDetection();

    public:

    protected:

    protected:
        void lineFit(const PointCloudTPtr cloudIn, float distThreshold,
                     PointCloudTPtr &cloudLine, PointCloudTPtr &cloudNonLine, pcl::ModelCoefficients::Ptr &coefficients);

        bool lineNormalFit(const PointCloudTPtr cloudIn, float distThreshold, PointT direct, LineInfo &lineInfo);

        double twoVectorAngle(PointT direct1, PointT direct2);

        double twoDirectSame(LineInfo lineInfo1, LineInfo lineInfo2);

        double dist2Point(PointT p1, PointT p2);

        double distOfPoint2Line(PointT p, LineInfo lineInfo);

        /*
         * 最小二乘法拟合直线 y = k0 + k1 * x
         * x[] 拟合数据的x向量
         * y[] 拟合数据的y向量
         * k[] 拟合后的系数向量 k0 = k[0], k1 = k[1]
         * */
        void fitLineMethod(PointCloudTPtr _cloud, std::vector<double> &k, PointT &direct);

        bool twoLinesIntersection(LineInfo line1, LineInfo line2, PointT &intersection);

        void polarFilter(PointCloudTPtr cloudIn, float angleResolution, PointCloudTPtr &cloudOut);

        unsigned int compute3DCentroid(PointCloudTPtr cloudIn, PointT &centroid);

        void getMaxMin(PointCloudTPtr _inputCloud, PointT &_minP, PointT &_maxP);

        void getZMaxMin(PointCloudTPtr _inputCloud, PointT &_zMinP, PointT &_zMaxP);

        void gridSegment(PointCloudTPtr _cloudIn, RectangleInfo _rectangleInfo, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec);

        void gridSegment(PointCloudTPtr _cloudIn, std::vector<PointT> _rectangleInfo, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec);

        void gridSegment(PointCloudTPtr _cloudIn, std::vector<cv::Point2d> polygon, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec);

        void gridSegment(PointCloudTPtr _cloudIn, std::string rectangleROIFile, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec);
    };

} // AutomatedPort
