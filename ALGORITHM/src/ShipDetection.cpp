
#include "ShipDetection.h"


namespace AutomatedPort {
    ShipDetection::ShipDetection() {
        shipParamIn = {};
        shipParamOut = {};
    }

    ShipDetection::~ShipDetection() {
        shipParamIn = {};
        shipParamOut = {};
    }

    void ShipDetection::runDetect() {
        this->init();
        this->runHatchDetect();
        this->runHatchBottomDetect();
    }

    void ShipDetection::init() {
        rtmethod::RTMethod rtMethod;
        Eigen::Matrix4d rtMat = Eigen::Matrix4d::Identity();
        rtMethod.rtStruct2RTMat_yxz(shipParamIn.rtParams, rtMat);
        PointCloudTPtr cloud_srcRT(new PointCloudT);
        pcl::transformPointCloud(*shipParamIn.cloudSrc, *cloud_srcRT, rtMat);

        PointCloudTPtr cloudSrc(new PointCloudT);
        roifilter::ROIFilter roiFilter;// 提取区域范围
        roiFilter.roiFilter(cloud_srcRT, shipParamIn.roiFile, 0.1, cloudSrc);
        *shipParamIn.cloudInited = *cloudSrc;
    }

    void ShipDetection::runHatchDetect() {
        shipParamOut.allClusters.clear();
        shipParamOut.rectangleInfoVec.clear();
        //bottomHighestPoint.clear();
        double st = Util::timeMs();
        this->estimateDeck();
//        cout << "estimateDeck time = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

//        cout << "deckProjectionCloud->size()=" << deckProjectionCloud->size() << endl;
        //this->estimateBordersByNormal(deckProjectionCloud, boundRadius, normalRadius, boundaryCloud);
        //this->estimateBordersByAlphaShape(deckProjectionCloud, 0.1, boundaryCloud);
//        cout << "estimateBorders time = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();
//        cout << "boundaryCloud->size()=" << boundaryCloud->size() << endl;

        this->polarFilter(shipParamOut.deckProjectionCloud, 2, shipParamOut.polarCloud);
//        cout << "polarFilter time = " << Util::timeMs() - st << " ms , polarCloud = " << shipParamOut.polarCloud->size() << endl;
//        st = Util::timeMs();

        lshapedfit::LShapedFIT lShapedFit;
        lshapedfit::ObjectInfo clusters{};//检测的目标尺寸信息
        lShapedFit.fitBox(shipParamOut.polarCloud, clusters);
        //cout<<clusters.direct.x<<" , "<<clusters.direct.y<<" , "<<clusters.direct.z<<endl;
        RectangleInfo rectangleInfo{};
        if (rectangleFit(shipParamOut.polarCloud, 0.3, clusters, rectangleInfo)) {
            shipParamOut.rectangleInfoVec.push_back(rectangleInfo);
            shipParamOut.allClusters.push_back(clusters);
            cout << "l = " << rectangleInfo.l << " , w = " << rectangleInfo.w << endl;
        }
//        cout << "rectangleFit time = " << Util::timeMs() - st << " ms " << endl;
        st = Util::timeMs();
    }

    void ShipDetection::runHatchBottomDetect() {
        double st = Util::timeMs();
        shipParamOut.bottomHighestPoint.clear();
        std::vector<cv::Point2d> polygon;
        if (!shipParamOut.rectangleInfoVec.empty()) {
            RectangleInfo rectangleInfo = shipParamOut.rectangleInfoVec.at(0);
            for (int j = 0; j < 4; ++j) {
                cv::Point2d point2D(rectangleInfo.cornerPoints.at(j).x, rectangleInfo.cornerPoints.at(j).y);
                polygon.push_back(point2D);
            }
        }

        if (polygon.empty()) {
            return;
        }

        // 降采样
        voxelgrid::VoxelGrid voxelGrid;
        voxelgrid::PointCloudTPtr voxelCloudRT(new voxelgrid::PointCloudT);
        voxelGrid.filter(shipParamOut.cloudSrcRT, 0.2, 0.2, 0.2, voxelCloudRT);

//        cout << "voxelGrid time = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        PointCloudTPtr cloudROI(new PointCloudT);
        roifilter::ROIFilter roiFilter;
        roiFilter.roiFilter(voxelCloudRT, polygon, 0.15, cloudROI);

//        cout << "roiFilter time = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        PointCloudTPtr cloudROIBottom(new PointCloudT);
        for (int i = 0; i < cloudROI->size(); ++i) {
            if (cloudROI->at(i).z < shipParamOut.deckPlaneParams.centreOfGravity.z + 1) {
                cloudROIBottom->push_back(cloudROI->at(i));
            }
        }

//        cout << "cloudROIBottom time = " << Util::timeMs() - st << " ms " << endl;
        st = Util::timeMs();

        dbscan::DBSCAN dbscan(4, 0.5, cloudROIBottom);
        dbscan.runCluster();
        vector<PointCloudTPtr> clusteredCloud = dbscan.getClusteredCloud();

//        cout << "dbscan time = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        int maxCloudNum = 0;
        int maxCloudIndex = -1;
        for (int i = 0; i < clusteredCloud.size(); ++i) {
            if (clusteredCloud.at(i)->size() > maxCloudNum) {
                maxCloudIndex = i;
                maxCloudNum = clusteredCloud.at(i)->size();
            }
        }

        if (maxCloudIndex >= 0) {
            // 网格等分
            vector<RectangleGrid> gridVec;
            this->gridSegment(clusteredCloud.at(maxCloudIndex), shipParamOut.rectangleInfoVec.at(0), shipParamIn.gridNumX, shipParamIn.gridNumY, gridVec);
//            cout << "gridVec = " << gridVec.size() << endl;
            shipParamOut.rectangleGridVec.clear();
            shipParamOut.rectangleGridVec = gridVec;
            for (int i = 0; i < gridVec.size(); ++i) {
                RectangleGrid &grid = gridVec.at(i);
                /*bottomHighestPoint.push_back(grid.centerPoint);
                for (int j = 0; j < grid.cornerPoints.size(); ++j) {
                    bottomHighestPoint.push_back(grid.cornerPoints.at(j));
                }*/
                shipParamOut.bottomHighestPoint.push_back(grid.highestPoint);
                shipParamOut.bottomHighestPoint.push_back(grid.lowestPoint);
            }

            /*PointT highestPoint = clusteredCloud.at(maxCloudIndex)->at(0);
            for (int i = 0; i < clusteredCloud.at(maxCloudIndex)->size(); ++i) {
                PointT &p = clusteredCloud.at(maxCloudIndex)->at(i);
                if (p.z > highestPoint.z) {
                    highestPoint = p;
                }
            }
            bottomHighestPoint.push_back(highestPoint);
            cout << "highestPoint = " << highestPoint.x << " , " << highestPoint.y << " , " << highestPoint.z << endl;*/
        }

//        cout << "highestPoint time = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

    }

    bool ShipDetection::estimateDeck() {
        shipParamOut.deckCloud->clear();
        shipParamOut.deckProjectionCloud->clear();
        // 降采样
        voxelgrid::VoxelGrid voxelGrid;
        voxelgrid::PointCloudTPtr voxelCloudOut(new voxelgrid::PointCloudT);
        voxelGrid.filter(shipParamIn.cloudInited, 0.5, 0.5, 0.5, voxelCloudOut);

        planemethod::PlaneMethod planeMethod;
        planemethod::PlaneParams planeParams{};
        planeMethod.calcNormalPlane(voxelCloudOut, 0.2, Eigen::Vector3f(0, 0, 1), planeParams);

        planemethod::PlaneParams earthPlaneParam{};
        earthPlaneParam.A = 0;
        earthPlaneParam.B = 0;
        earthPlaneParam.C = 1;
        earthPlaneParam.D = planeParams.D / planeParams.C;
        Eigen::Matrix4d rtMat = Eigen::Matrix4d::Identity();
        Eigen::Vector4d projectionPoint = Eigen::Vector4d::Identity();
        planeMethod.calcRotationBy2Plane(planeParams, earthPlaneParam, rtMat, projectionPoint);
        rtmethod::RTMethod rtMethod;
        rtmethod::RTParams rtParams{};
        rtMethod.rtMat2RTStruct_yxz(rtMat, rtParams);

        /*PointCloudTPtr voxelCloudRT(new PointCloudT);
        pcl::transformPointCloud(*voxelCloudOut, *voxelCloudRT, rtMat);
        planemethod::PlaneMethod planeMethod1;
        planemethod::PlaneParams planeParams1{};
        planeMethod1.calcNormalPlane(voxelCloudRT, 0.2, Eigen::Vector3f(0, 0, 1), planeParams1);
        deckPlaneParams = planeParams1;*/

        shipParamOut.deckPlaneParams = planeParams;
        shipParamOut.deckPlaneParams.centreOfGravity.z = -earthPlaneParam.D;
        shipParamOut.deckPlaneParams.A = 0;
        shipParamOut.deckPlaneParams.B = 0;
        shipParamOut.deckPlaneParams.C = 1;
        shipParamOut.deckPlaneParams.D = planeParams.C * planeParams.D;
        shipParamOut.deckPlaneParams.coefficients->values.at(0) = shipParamOut.deckPlaneParams.A;
        shipParamOut.deckPlaneParams.coefficients->values.at(1) = shipParamOut.deckPlaneParams.B;
        shipParamOut.deckPlaneParams.coefficients->values.at(2) = shipParamOut.deckPlaneParams.C;
        shipParamOut.deckPlaneParams.coefficients->values.at(3) = shipParamOut.deckPlaneParams.D;

        PointCloudTPtr cloudRT(new PointCloudT);
        pcl::transformPointCloud(*shipParamIn.cloudInited, *cloudRT, rtMat);
        *shipParamOut.cloudSrcRT = *cloudRT;
        for (int i = 0; i < cloudRT->size(); ++i) {
            if (cloudRT->at(i).z > shipParamOut.deckPlaneParams.centreOfGravity.z - 0.5) {
                shipParamOut.deckCloud->push_back(cloudRT->at(i));
            } else {
                shipParamOut.deckBottomCloud->push_back(cloudRT->at(i));
            }
        }
//        cout << "deckCloud = " << deckCloud->size() << " , cloudRT = " << cloudRT->size() << " , deckBottomCloud = " << deckBottomCloud->size() << endl;

        PointCloudTPtr deckProjectionCloudTmp(new PointCloudT);
        int ip = 0;
        for (auto p: *shipParamOut.deckCloud) {
            p.z = (ip % 2) * 0.002 + shipParamOut.deckPlaneParams.centreOfGravity.z;//(ip % 2) * 0.002 +
            deckProjectionCloudTmp->push_back(p);
            ip++;
        }

        voxelGrid.filter(deckProjectionCloudTmp, 0.1, 0.1, 0.1, shipParamOut.deckProjectionCloud);

        return false;
    }

    int ShipDetection::estimateBordersByNormal(PointCloudTPtr &cloud, float _boundRadius, float _normalRadius, PointCloudTPtr &cloudOut) {
        pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
        pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
        pcl::NormalEstimationOMP<PointT, pcl::Normal> normEst; //定义一个法线估计的对象
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
        //PointCloudTPtr cloud_boundary(new PointCloudT);
        double st = Util::timeMs();
        normEst.setInputCloud(PointCloudTPtr(cloud));
        normEst.setRadiusSearch(_normalRadius); //设置法线估计的半径//normEst.setKSearch(10);//表示计算点云法向量时，搜索的点云个数
        normEst.setNumberOfThreads(30);
        normEst.compute(*normals); //将法线估计结果保存至normals

        cout << "       normEst time = " << Util::timeMs() - st << " ms " << endl;
        st = Util::timeMs();
        //输出法线的个数
        //std::cout << "normalRadius: " << normalRadius << std::endl;

        boundEst.setInputCloud(cloud); //设置输入的点云
        boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
        //boundEst.setRadiusSearch(_boundRadius); //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
        boundEst.setKSearch(200);
        boundEst.setAngleThreshold(M_PI / 3); //边界估计时的角度阈值M_PI / 4  并计算k邻域点的法线夹角,若大于阈值则为边界特征点
        boundEst.setSearchMethod(pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>)); //设置搜索方式KdTree
        boundEst.compute(boundaries); //将边界估计结果保存在boundaries

        cout << "       boundEst time = " << Util::timeMs() - st << " ms " << endl;
        st = Util::timeMs();

        //std::cerr << "AngleThreshold: " << M_PI / 4 << std::endl;
        //输出边界点的个数
        //std::cerr << "boundaries: " << boundaries.points.size() << std::endl;
        //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
        for (int i = 0; i < cloud->points.size(); i++) {
            if (boundaries[i].boundary_point > 0) {
                cloudOut->push_back(cloud->points[i]);
            }
        }
        return 0;
    }

    int ShipDetection::estimateBordersByAlphaShape(PointCloudTPtr &_cloud, float _alpha, PointCloudTPtr &_cloudOut) {
        pcl::ConcaveHull<PointT> concaveHull;
        concaveHull.setAlpha(_alpha);
        concaveHull.setInputCloud(_cloud);
        concaveHull.reconstruct(*_cloudOut);

        return 0;
    }

    bool ShipDetection::rectangleFit(PointCloudTPtr cloud, float distThreshold, lshapedfit::ObjectInfo clusters, RectangleInfo &rectangleInfo) {
        rectangleInfo = {};
        vector<LineInfo> lineInfoVecTmp;
        lineInfoVecTmp.resize(4);
        //带法线的直线拟合
        if (!lineNormalFit(cloud, distThreshold, clusters.direct, lineInfoVecTmp.at(0))) {
            return false;
        }

        if (!lineNormalFit(lineInfoVecTmp.at(0).nonLineCloud, distThreshold, clusters.direct, lineInfoVecTmp.at(1))) {
            return false;
        }
        //判断2条拟合的直线方向向量的方向是否一致，即点积大于0，如果方向相反，则取反一条线方向向量
        if (twoDirectSame(lineInfoVecTmp.at(0), lineInfoVecTmp.at(1)) < 0) {
            for (int j = 0; j < 3; ++j) {
                lineInfoVecTmp.at(1).coefficients->values.at(3 + j) *= -1.0;
            }
        }
        PointT directTmp;//平均平行线向量值
        directTmp.x = (lineInfoVecTmp.at(0).coefficients->values.at(3) + lineInfoVecTmp.at(1).coefficients->values.at(3)) * 0.5f;
        directTmp.y = (lineInfoVecTmp.at(0).coefficients->values.at(4) + lineInfoVecTmp.at(1).coefficients->values.at(4)) * 0.5f;
        directTmp.z = (lineInfoVecTmp.at(0).coefficients->values.at(5) + lineInfoVecTmp.at(1).coefficients->values.at(5)) * 0.5f;
        lineInfoVecTmp.at(0).coefficients->values.at(3) = directTmp.x;
        lineInfoVecTmp.at(0).coefficients->values.at(4) = directTmp.y;
        lineInfoVecTmp.at(0).coefficients->values.at(5) = directTmp.z;
        lineInfoVecTmp.at(1).coefficients->values.at(3) = directTmp.x;
        lineInfoVecTmp.at(1).coefficients->values.at(4) = directTmp.y;
        lineInfoVecTmp.at(1).coefficients->values.at(5) = directTmp.z;

        lineInfoVecTmp.at(0).direct = directTmp;
        lineInfoVecTmp.at(0).centerPoint.x = lineInfoVecTmp.at(0).coefficients->values.at(0);
        lineInfoVecTmp.at(0).centerPoint.y = lineInfoVecTmp.at(0).coefficients->values.at(1);
        lineInfoVecTmp.at(0).centerPoint.z = lineInfoVecTmp.at(0).coefficients->values.at(2);
        lineInfoVecTmp.at(1).direct = directTmp;
        lineInfoVecTmp.at(1).centerPoint.x = lineInfoVecTmp.at(1).coefficients->values.at(0);
        lineInfoVecTmp.at(1).centerPoint.y = lineInfoVecTmp.at(1).coefficients->values.at(1);
        lineInfoVecTmp.at(1).centerPoint.z = lineInfoVecTmp.at(1).coefficients->values.at(2);

        // 将相对的平行线放入结构体
        rectangleInfo.linesL.push_back(lineInfoVecTmp.at(0));
        rectangleInfo.linesL.push_back(lineInfoVecTmp.at(1));

        if (!lineNormalFit(lineInfoVecTmp.at(1).nonLineCloud, distThreshold, clusters.directW, lineInfoVecTmp.at(2))) {
            return false;
        }

        if (!lineNormalFit(lineInfoVecTmp.at(2).nonLineCloud, distThreshold, clusters.directW, lineInfoVecTmp.at(3))) {
            return false;
        }
        if (twoDirectSame(lineInfoVecTmp.at(2), lineInfoVecTmp.at(3)) < 0) {
            for (int j = 0; j < 3; ++j) {
                lineInfoVecTmp.at(3).coefficients->values.at(3 + j) *= -1.0;
            }
        }
        directTmp.x = (lineInfoVecTmp.at(2).coefficients->values.at(3) + lineInfoVecTmp.at(3).coefficients->values.at(3)) * 0.5f;
        directTmp.y = (lineInfoVecTmp.at(2).coefficients->values.at(4) + lineInfoVecTmp.at(3).coefficients->values.at(4)) * 0.5f;
        directTmp.z = (lineInfoVecTmp.at(2).coefficients->values.at(5) + lineInfoVecTmp.at(3).coefficients->values.at(5)) * 0.5f;
        lineInfoVecTmp.at(2).coefficients->values.at(3) = directTmp.x;
        lineInfoVecTmp.at(2).coefficients->values.at(4) = directTmp.y;
        lineInfoVecTmp.at(2).coefficients->values.at(5) = directTmp.z;
        lineInfoVecTmp.at(3).coefficients->values.at(3) = directTmp.x;
        lineInfoVecTmp.at(3).coefficients->values.at(4) = directTmp.y;
        lineInfoVecTmp.at(3).coefficients->values.at(5) = directTmp.z;

        lineInfoVecTmp.at(2).direct = directTmp;
        lineInfoVecTmp.at(2).centerPoint.x = lineInfoVecTmp.at(2).coefficients->values.at(0);
        lineInfoVecTmp.at(2).centerPoint.y = lineInfoVecTmp.at(2).coefficients->values.at(1);
        lineInfoVecTmp.at(2).centerPoint.z = lineInfoVecTmp.at(2).coefficients->values.at(2);
        lineInfoVecTmp.at(3).direct = directTmp;
        lineInfoVecTmp.at(3).centerPoint.x = lineInfoVecTmp.at(3).coefficients->values.at(0);
        lineInfoVecTmp.at(3).centerPoint.y = lineInfoVecTmp.at(3).coefficients->values.at(1);
        lineInfoVecTmp.at(3).centerPoint.z = lineInfoVecTmp.at(3).coefficients->values.at(2);

        rectangleInfo.linesW.push_back(lineInfoVecTmp.at(2));
        rectangleInfo.linesW.push_back(lineInfoVecTmp.at(3));

        double twoLinesAngle = this->twoVectorAngle(rectangleInfo.linesL.at(0).direct, rectangleInfo.linesW.at(0).direct);

        // 计算两两直线的交点
        rectangleInfo.cornerPoints.resize(4);
        bool cornerRet0 = twoLinesIntersection(rectangleInfo.linesL.at(0), rectangleInfo.linesW.at(0), rectangleInfo.cornerPoints.at(0));
        bool cornerRet1 = twoLinesIntersection(rectangleInfo.linesL.at(0), rectangleInfo.linesW.at(1), rectangleInfo.cornerPoints.at(1));
        bool cornerRet2 = twoLinesIntersection(rectangleInfo.linesL.at(1), rectangleInfo.linesW.at(1), rectangleInfo.cornerPoints.at(2));
        bool cornerRet3 = twoLinesIntersection(rectangleInfo.linesL.at(1), rectangleInfo.linesW.at(0), rectangleInfo.cornerPoints.at(3));

        double d1 = this->dist2Point(rectangleInfo.cornerPoints.at(0), rectangleInfo.cornerPoints.at(1));
        double d2 = this->dist2Point(rectangleInfo.cornerPoints.at(1), rectangleInfo.cornerPoints.at(2));
        double d3 = this->dist2Point(rectangleInfo.cornerPoints.at(2), rectangleInfo.cornerPoints.at(3));
        double d4 = this->dist2Point(rectangleInfo.cornerPoints.at(3), rectangleInfo.cornerPoints.at(0));
//        cout << "d = " << d1 << " , " << d2 << " , " << d3 << " , " << d4 << " , twoLinesAngle = " << twoLinesAngle << endl;
        rectangleInfo.l = d1;
        rectangleInfo.w = d2;
        if (twoLinesAngle < 88) {
            //cout << "d = " << d1 << " , " << d2 << " , " << d3 << " , " << d4 << " , twoLinesAngle = " << twoLinesAngle << endl;
            return false;
        }

        return true;
    }

    //******************************getter & setter******************************************************

    const ShipParamIn &ShipDetection::getShipParamIn() const {
        return shipParamIn;
    }

    void ShipDetection::setShipParamIn(const ShipParamIn &_shipParamIn) {
        ShipDetection::shipParamIn = _shipParamIn;
    }

    const ShipParamOut &ShipDetection::getShipParamOut() const {
        return shipParamOut;
    }

    void ShipDetection::setShipParamOut(const ShipParamOut &_shipParamOut) {
        ShipDetection::shipParamOut = _shipParamOut;
    }


} // ship_cloud
