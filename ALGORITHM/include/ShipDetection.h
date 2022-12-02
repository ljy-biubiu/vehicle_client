#pragma once

#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include "dbscan.h"
#include "LShapedFitting.h"
#include "VoxelGrid.h"
#include "Util.h"
#include "PlaneMethod.h"
#include "RTMethod.h"
#include "BaseDetection.h"

using namespace std;

namespace AutomatedPort {

struct ShipParamIn {
    PointCloudTPtr cloudSrc;
    float boundRadius;//边界提取的半径
    float normalRadius;//法线提取的半径
    int gridNumX;//
    int gridNumY;
    double max_monitor_angle;
    double min_monitor_angle;
    rtmethod::RTParams rtParams;
    std::string roiFile;
    PointCloudTPtr cloudInited;

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

struct ShipParamOut {
    PointCloudTPtr cloudSrcRT;//原始校正点云
    planemethod::PlaneParams deckPlaneParams;// 甲板平面参数
    PointCloudTPtr deckCloud;// 甲板点云
    PointCloudTPtr deckBottomCloud;// 甲板底部点云
    PointCloudTPtr deckProjectionCloud;// 甲板投影点云
    PointCloudTPtr boundaryCloud;//边界点云
    vector<RectangleInfo> rectangleInfoVec;// 识别后的矩形信息
    vector<lshapedfit::ObjectInfo> allClusters;// 所有聚类结果信息
    std::vector<PointT> bottomHighestPoint;// 底部最高点
    PointCloudTPtr polarCloud;// 极坐标点云
    vector<RectangleGrid> rectangleGridVec;// 每个矩形栅格信息

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

class ShipDetection : public BaseDetection {
public:
    ShipDetection();

    ~ShipDetection();

public:
    void runDetect();

public:
    const ShipParamIn &getShipParamIn() const;

    void setShipParamIn(const ShipParamIn &shipParamIn);

    const ShipParamOut &getShipParamOut() const;

    void setShipParamOut(const ShipParamOut &shipParamOut);

private:
    ShipParamIn shipParamIn;
    ShipParamOut shipParamOut;
private:
    bool estimateDeck();

    int estimateBordersByNormal(PointCloudTPtr &cloud, float _boundRadius, float _normalRadius, PointCloudTPtr &cloudOut);

    int estimateBordersByAlphaShape(PointCloudTPtr &_cloud, float _alpha, PointCloudTPtr &_cloudOut);

    bool rectangleFit(PointCloudTPtr cloud, float distThreshold, lshapedfit::ObjectInfo clusters, RectangleInfo &rectangleInfo);

    void init();

    void runHatchDetect();

    void runHatchBottomDetect();
};

} // ship_cloud
