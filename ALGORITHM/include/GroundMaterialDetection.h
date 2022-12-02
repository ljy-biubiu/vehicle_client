#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "dbscan.h"
#include "LShapedFitting.h"
#include "VoxelGrid.h"
#include "Util.h"
#include "PlaneMethod.h"
#include "RTMethod.h"
#include "ROIFilter.h"
#include "CSF.h"
#include "BaseDetection.h"

using namespace std;

namespace AutomatedPort {
    struct GroundParamIn {
        PointCloudTPtr cloudSrc;
        int gridNumX;//
        int gridNumY;
        double min_monitor_angle;
        double max_monitor_angle;
        rtmethod::RTParams rtParams;
        std::string roiFile;
        std::string rectangleROIFile;//点顺序：左下角，右下角，右上角，左上角
        PointCloudTPtr cloudInited;

        GroundParamIn() {
            cloudSrc.reset(new PointCloudT);
            gridNumX = 0;
            gridNumY = 0;
            min_monitor_angle = 0;
            max_monitor_angle = 0;
            rtParams = {};
            roiFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/GroundROIPoints.txt";
            rectangleROIFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/GroundRectanglePoints.txt";
            cloudInited.reset(new PointCloudT);
        }
    };

    struct GroundParamOut {
        PointCloudTPtr cloudSrcRT;
        planemethod::PlaneParams groundPlaneParams;
        PointCloudTPtr groundCloud;
        std::vector<PointT> materialHighestPoint;

        GroundParamOut() {
            cloudSrcRT.reset(new PointCloudT);
            groundCloud.reset(new PointCloudT);
            groundPlaneParams = {};
        }
    };

    class GroundMaterialDetection : public BaseDetection {
    public:
        GroundMaterialDetection();

        ~GroundMaterialDetection();

    public:
        void runDetect();

    public:
        const GroundParamIn &getGroundParamIn() const;

        void setGroundParamIn(const GroundParamIn &paramIn);

        const GroundParamOut &getGroundParamOut() const;

        void setGroundParamOut(const GroundParamOut &paramOut);

    private:
        GroundParamIn paramIn;
        GroundParamOut paramOut;

    private:
        void init();

        void csfSegment(PointCloudTPtr cloudIn, PointCloudTPtr &cloudGround, PointCloudTPtr &cloudNonGround);

        void gridSeg(PointCloudTPtr _inputCloud, PointT &highestPoint);
    };

} // AutomatedPort
