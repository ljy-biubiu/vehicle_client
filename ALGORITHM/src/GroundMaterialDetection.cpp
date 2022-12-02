//
// Created by niujy on 2022/11/23.
//

#include "GroundMaterialDetection.h"

namespace AutomatedPort {
    GroundMaterialDetection::GroundMaterialDetection() {
        paramIn = {};
        paramOut = {};
    }

    GroundMaterialDetection::~GroundMaterialDetection() {
        paramIn = {};
        paramOut = {};
    }

    void GroundMaterialDetection::init() {
        rtmethod::RTMethod rtMethod;
        Eigen::Matrix4d rtMat = Eigen::Matrix4d::Identity();
        rtMethod.rtStruct2RTMat_yxz(paramIn.rtParams, rtMat);
        PointCloudTPtr cloud_srcRT(new PointCloudT);
        pcl::transformPointCloud(*paramIn.cloudSrc, *cloud_srcRT, rtMat);

        PointCloudTPtr cloudSrc(new PointCloudT);
        roifilter::ROIFilter roiFilter;// 提取区域范围
        roiFilter.roiFilter(cloud_srcRT, paramIn.roiFile, 0.1, cloudSrc);
        *paramIn.cloudInited = *cloudSrc;
    }

    void GroundMaterialDetection::runDetect() {
        this->init();

        double st = Util::timeMs();
        // 降采样
        voxelgrid::VoxelGrid voxelGrid;
        voxelgrid::PointCloudTPtr voxelCloudOut(new voxelgrid::PointCloudT);
        voxelGrid.filter(paramIn.cloudInited, 0.5, 0.5, 0.5, voxelCloudOut);

//        cout << "       voxelGrid = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        planemethod::PlaneMethod planeMethod;
        planemethod::PlaneParams planeParams{};
        planeMethod.calcNormalPlane(voxelCloudOut, 0.2, Eigen::Vector3f(0, 0, 1), planeParams);
        //planeMethod.calcPlane(voxelCloudOut, 0.2, planeParams);
        paramOut.groundPlaneParams = planeParams;

//        cout << "       planeMethod = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

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

        paramOut.groundPlaneParams = planeParams;
        paramOut.groundPlaneParams.centreOfGravity.z = -earthPlaneParam.D;
        paramOut.groundPlaneParams.A = 0;
        paramOut.groundPlaneParams.B = 0;
        paramOut.groundPlaneParams.C = 1;
        paramOut.groundPlaneParams.D = planeParams.C * planeParams.D;
        paramOut.groundPlaneParams.coefficients->values.at(0) = paramOut.groundPlaneParams.A;
        paramOut.groundPlaneParams.coefficients->values.at(1) = paramOut.groundPlaneParams.B;
        paramOut.groundPlaneParams.coefficients->values.at(2) = paramOut.groundPlaneParams.C;
        paramOut.groundPlaneParams.coefficients->values.at(3) = paramOut.groundPlaneParams.D;

        PointCloudTPtr cloudRT(new PointCloudT);
        pcl::transformPointCloud(*paramIn.cloudInited, *cloudRT, rtMat);
        *paramOut.cloudSrcRT = *cloudRT;

//        cout << "       transformPointCloud = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        voxelgrid::PointCloudTPtr voxelCloudOut1(new voxelgrid::PointCloudT);
        voxelGrid.filter(paramIn.cloudInited, 0.1, 0.1, 0.1, voxelCloudOut1);
        PointCloudTPtr voxelCloudRT(new PointCloudT);
        pcl::transformPointCloud(*voxelCloudOut1, *voxelCloudRT, rtMat);

//        cout << "       voxelGrid = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        PointCloudTPtr cloudGround(new PointCloudT);
        PointCloudTPtr cloudNonGround(new PointCloudT);
        this->csfSegment(voxelCloudRT, cloudGround, cloudNonGround);

//        cout << "       csfSegment = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        *paramOut.groundCloud = *cloudGround;
//        cout << "       voxelCloudRT = " << voxelCloudRT->size() << " , cloudGround = " << cloudGround->size() << " , cloudNonGround = " << cloudNonGround->size()
//             << endl;

        paramOut.materialHighestPoint.clear();
        vector<RectangleGrid> rectangleGridVec;
        this->gridSegment(cloudGround, paramIn.rectangleROIFile, paramIn.gridNumX, paramIn.gridNumY, rectangleGridVec);
        for (int i = 0; i < rectangleGridVec.size(); ++i) {
            RectangleGrid &grid = rectangleGridVec.at(i);
            /*materialHighestPoint.push_back(grid.centerPoint);
            for (int j = 0; j < grid.cornerPoints.size(); ++j) {
                materialHighestPoint.push_back(grid.cornerPoints.at(j));
            }*/
            paramOut.materialHighestPoint.push_back(grid.highestPoint);
            //materialHighestPoint.push_back(grid.lowestPoint);
        }

        /*PointT highestPoint = cloudGround->at(0);
        for (auto &p: *cloudGround) {
            if (p.z > highestPoint.z) {
                highestPoint = p;
            }
        }*/
//        cout << "       highestPoint = " << highestPoint.x << " , " << highestPoint.y << " , " << highestPoint.z << endl;

//        cout << "       highestPoint = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        //this->gridSeg(cloudGround, highestPoint);

//        cout << "       gridSeg = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

//        materialHighestPoint.clear();
//        materialHighestPoint.push_back(highestPoint);
    }

    void GroundMaterialDetection::csfSegment(PointCloudTPtr cloudIn, PointCloudTPtr &cloudGround, PointCloudTPtr &cloudNonGround) {
        cloudGround.reset(new PointCloudT);
        cloudNonGround.reset(new PointCloudT);
        CSF csf;
        std::vector<csf::Point> csfCloud;
        for (size_t i = 0; i < cloudIn->size(); i++) {
            csf::Point csfP;
            csfP.x = cloudIn->points[i].x;
            csfP.y = cloudIn->points[i].y;
            csfP.z = cloudIn->points[i].z;
            csfCloud.push_back(csfP);
        }
        //用于布料算法处理的数据
        csf.setPointCloud(csfCloud);
        csf.params.iterations = cloudIn->size();
        //每次进来初始化一次
        std::vector<int> groundIndexes, offGroundIndexes;
        csf.do_filtering(groundIndexes, offGroundIndexes, false);
        for (int groundIndex: groundIndexes) {
            cloudGround->push_back(cloudIn->at(groundIndex));
        }
        for (int offGroundIndex: offGroundIndexes) {
            cloudNonGround->push_back(cloudIn->at(offGroundIndex));
        }
    }

    void GroundMaterialDetection::gridSeg(PointCloudTPtr _inputCloud, PointT &highestPoint) {
        PointT minP;
        PointT maxP;
        this->getMaxMin(_inputCloud, minP, maxP);
        PointT cellGrid;
        cellGrid.x = 0.3f;
        cellGrid.y = 0.3f;
        cellGrid.z = 0.5f;
        GridCell divSize{};
        if (cellGrid.x > 0 && cellGrid.y > 0 && cellGrid.z > 0) {
            divSize.ix = ceil((maxP.x - minP.x) / cellGrid.x);
            divSize.jy = ceil((maxP.y - minP.y) / cellGrid.y);
            divSize.kz = ceil((maxP.z - minP.z) / cellGrid.z);
        }
        unsigned long totalGridNum = divSize.ix * divSize.jy * divSize.kz;

        vector<int> numVec(totalGridNum, 0);
        vector<PointT> pVec(totalGridNum);

        for (auto &p: *_inputCloud) {
            if (p.x < minP.x || p.x > maxP.x || p.y < minP.y || p.y > maxP.y || p.z < minP.z || p.z > maxP.z) {
                continue;
            }
            GridCell cell{};
            cell.ix = (int) floor((p.x - minP.x) / cellGrid.x);
            cell.jy = (int) floor((p.y - minP.y) / cellGrid.y);
            cell.kz = (int) floor((p.z - minP.z) / cellGrid.z);
            cell.id = cell.ix + cell.jy * divSize.ix + cell.kz * divSize.ix * divSize.jy;
            if (cell.ix >= divSize.ix || cell.jy >= divSize.jy || cell.kz >= divSize.kz) {
                continue;
            }

            PointT tmpP{};
            tmpP.x = (pVec[cell.id].x * numVec[cell.id] + p.x) / (numVec[cell.id] + 1);
            tmpP.y = (pVec[cell.id].y * numVec[cell.id] + p.y) / (numVec[cell.id] + 1);
            tmpP.z = (pVec[cell.id].z * numVec[cell.id] + p.z) / (numVec[cell.id] + 1);
            //cout<<"tmpP = "<<tmpP.x<<" , "<<tmpP.y<<" , "<<tmpP.z<<endl;
            numVec[cell.id]++;
            pVec[cell.id] = tmpP;
        }
        highestPoint.z = -9999;
        for (auto p: pVec) {
            if (highestPoint.z < p.z && p.z < 0) {
                highestPoint = p;
            }
        }
//        cout << "highestPoint = " << highestPoint.x << " , " << highestPoint.y << " , " << highestPoint.z << endl;
    }

    const GroundParamIn &GroundMaterialDetection::getGroundParamIn() const {
        return paramIn;
    }

    void GroundMaterialDetection::setGroundParamIn(const GroundParamIn &_paramIn) {
        GroundMaterialDetection::paramIn = _paramIn;
    }

    const GroundParamOut &GroundMaterialDetection::getGroundParamOut() const {
        return paramOut;
    }

    void GroundMaterialDetection::setGroundParamOut(const GroundParamOut &_paramOut) {
        GroundMaterialDetection::paramOut = _paramOut;
    }


} // AutomatedPort
