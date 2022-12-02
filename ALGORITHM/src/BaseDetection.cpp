//
// Created by niujy on 2022/11/28.
//

#include "BaseDetection.h"

namespace AutomatedPort {
    BaseDetection::BaseDetection() {

    }

    BaseDetection::~BaseDetection() {

    }

    void BaseDetection::lineFit(const PointCloudTPtr cloudIn, float distThreshold, PointCloudTPtr &cloudLine, PointCloudTPtr &cloudNonLine,
                                pcl::ModelCoefficients::Ptr &coefficients) {
        if (cloudIn->size() < 2) {
            return;
        }
        //cout << "cloudIn = " << cloudIn->size() << endl;
        //创建一个模型参数对象，用于记录结果
        //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        coefficients.reset(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
        pcl::SACSegmentation<PointT> seg;     // 创建一个分割器
        seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
        seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
        seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
        seg.setDistanceThreshold(distThreshold);         //设置误差容忍范围，也就是阈值
        seg.setMaxIterations(cloudIn->size());
        seg.setInputCloud(cloudIn);               //输入点云
        seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
        //cout << "cloudIn = " << seg.getInputCloud()->size() << endl;
        if (inliers->indices.size() == 0) {
            PCL_ERROR ("Could not estimate a line model for the given dataset.");
            return;
        }
        //打印直线方程
        /*std::cout << "a：" << coefficients->values.at(0) << endl;
        std::cout << "b：" << coefficients->values[1] << endl;
        std::cout << "c：" << coefficients->values[2] << endl;
        std::cout << "d：" << coefficients->values[3] << endl;
        std::cout << "e：" << coefficients->values[4] << endl;
        std::cout << "f：" << coefficients->values[5] << endl;*/

        /*单条直线时*/
        //直线点获取
        PointCloudTPtr c_plane2(new PointCloudT);  //存储直线点云
        pcl::ExtractIndices<PointT> extract;  //创建点云提取对象
        extract.setInputCloud(cloudIn);    //设置输入点云
        extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
        extract.setNegative(false);      //false提取内点, true提取外点
        extract.filter(*cloudLine);        //提取输出存储到c_plane2
        extract.setNegative(true);      //false提取内点, true提取外点
        extract.filter(*cloudNonLine);        //提取输出存储到c_plane2
    }

    bool BaseDetection::lineNormalFit(const PointCloudTPtr cloudIn, float distThreshold, PointT direct, LineInfo &lineInfo) {
        if (cloudIn->size() < 4) {
            return false;
        }
        //PCL直线拟合模型的BUG：当所有点z值一样时，无法使用直线模板进行匹配，报错“[pcl::RandomSampleConsensus::computeModel] No samples could be selected!”
        //故对其中一个点的z值进行手动微调，使存在差异。
        cloudIn->at(0).z += 0.1;
        //创建一个模型参数对象，用于记录结果
        PointCloudTPtr cloudLine(new PointCloudT);
        PointCloudTPtr cloudNonLine(new PointCloudT);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
        pcl::SACSegmentation<PointT> seg;     // 创建一个分割器
        seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
        seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);  // Mandatory-设置目标几何形状
        seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
        seg.setDistanceThreshold(distThreshold);         //设置误差容忍范围，也就是阈值
        seg.setAxis(Eigen::Vector3f(direct.x, direct.y, direct.z));
        seg.setEpsAngle(20 * M_PI / 180);
        seg.setMaxIterations(cloudIn->size());
        seg.setInputCloud(cloudIn);               //输入点云
        seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
        //cout << "cloudIn = " << seg.getInputCloud()->size() << endl;
        if (inliers->indices.size() == 0) {
            //PCL_ERROR ("Could not estimate a line model for the given dataset.");
            /*for (auto &p: *cloudIn) {
                cout << p.x << " , " << p.y << " , " << p.z << endl;
            }*/
            return false;
        }

        //直线点获取
        PointCloudTPtr c_plane2(new PointCloudT);  //存储直线点云
        pcl::ExtractIndices<PointT> extract;  //创建点云提取对象
        extract.setInputCloud(cloudIn);    //设置输入点云
        extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
        extract.setNegative(false);      //false提取内点, true提取外点
        extract.filter(*cloudLine);        //提取输出存储到c_plane2
        extract.setNegative(true);      //false提取内点, true提取外点
        extract.filter(*cloudNonLine);        //提取输出存储到c_plane2

        *lineInfo.cloud = *cloudLine;
        *lineInfo.nonLineCloud = *cloudNonLine;
        *lineInfo.coefficients = *coefficients;
        return true;
    }

    double BaseDetection::twoVectorAngle(PointT direct1, PointT direct2) {
        /*cout << "direct1 = " << direct1.x << " , " << direct1.y << " , " << direct1.z << " , direct2 = " << direct2.x << " , " << direct2.y << " , " << direct2.z
         << endl;*/
        double v1_v2 = abs(direct1.x * direct2.x + direct1.y * direct2.y + direct1.z * direct2.z);
        v1_v2 = v1_v2 >= 1 ? 1 : v1_v2;
        double theta = 180 / M_PI * acos(v1_v2);
        return theta;
    }

    double BaseDetection::twoDirectSame(LineInfo lineInfo1, LineInfo lineInfo2) {
        if (lineInfo1.coefficients->values.empty() || lineInfo2.coefficients->values.empty()) {
            return 0;
        }
        double value = lineInfo1.coefficients->values.at(3) * lineInfo2.coefficients->values.at(3)
                       + lineInfo1.coefficients->values.at(4) * lineInfo2.coefficients->values.at(4)
                       + lineInfo1.coefficients->values.at(5) * lineInfo2.coefficients->values.at(5);
        return value;
    }

    double BaseDetection::dist2Point(PointT p1, PointT p2) {
        double dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
        return dist;
    }

    double BaseDetection::distOfPoint2Line(PointT p, LineInfo lineInfo) {
        PointT s = lineInfo.direct;
        PointT v;
        v.x = p.x - lineInfo.centerPoint.x;
        v.y = p.y - lineInfo.centerPoint.y;
        v.z = p.z - lineInfo.centerPoint.z;
        /*cout << "p=" << p.x << " , " << p.y << " , " << p.z
             << " , direct=" << s.x << " , " << s.y << " , " << s.z
             << " , centerPoint=" << lineInfo.centerPoint.x << " , " << lineInfo.centerPoint.y << " , " << lineInfo.centerPoint.z << endl;*/
        double v2 = v.x * v.x + v.y * v.y + v.z * v.z;
        double sv = s.x * v.x + s.y * v.y + s.z * v.z;
        double s2 = s.x * s.x + s.y * s.y + s.z * s.z;
        double d2 = v2 - sv * sv / s2;
        d2 = d2 < 0 ? 0 : d2;
        double dist = sqrt(d2);
        return dist;
    }

    void BaseDetection::fitLineMethod(PointCloudTPtr _cloud, vector<double> &k, PointT &direct) {
        int N = _cloud->size();
        k.resize(2);
        double k1 = 0, k2 = 0, k3 = 0, k4 = 0, k5 = 0;
        k[0] = 0;
        k[1] = 0;
        for (int i = 0; i < N; i++) {
            PointT p = _cloud->at(i);
            k1 += p.x * p.x;
            k2 += p.x;
            k3 += p.x * p.y;
            k4 += 1;
            k5 += p.y;
        }
        double err = k2 * k2 - k1 * k4;
        if (abs(err) > 0.0005) {
            k[0] = (k2 * k3 - k1 * k5) / (k2 * k2 - k1 * k4);
            k[1] = (k2 * k5 - k3 * k4) / (k2 * k2 - k1 * k4);

            direct.x = 1;
            direct.y = k[0] + k[1] * direct.x;
            direct.z = 0;
            double dist = sqrt(direct.x * direct.x + direct.y * direct.y);
            direct.x /= dist;
            direct.y /= dist;
        } else {
            direct.x = 0;
            direct.y = 1;
            direct.z = 0;
        }
    }

    bool BaseDetection::twoLinesIntersection(LineInfo line1, LineInfo line2, PointT &intersection) {
        // 直线一般式方程 ax+by+c=0
        double a1 = line1.coefficients->values.at(4);
        double b1 = -line1.coefficients->values.at(3);
        double c1 = line1.coefficients->values.at(3) * line1.coefficients->values.at(1) - line1.coefficients->values.at(4) * line1.coefficients->values.at(0);

        double a2 = line2.coefficients->values.at(4);
        double b2 = -line2.coefficients->values.at(3);
        double c2 = line2.coefficients->values.at(3) * line2.coefficients->values.at(1) - line2.coefficients->values.at(4) * line2.coefficients->values.at(0);
        // D为0时，表示两直线平行
        double D = a1 * b2 - a2 * b1;
        if (D == 0) {
            intersection = {};
            return false;
        }
        // 交点坐标
        intersection.x = (b1 * c2 - b2 * c1) / D;
        intersection.y = (a2 * c1 - a1 * c2) / D;
        intersection.z = (line1.coefficients->values.at(2) + line2.coefficients->values.at(2)) * 0.5f;
        return true;
    }

    void BaseDetection::polarFilter(PointCloudTPtr cloudIn, float angleResolution, PointCloudTPtr &cloudOut) {
        if (cloudIn->empty()) {
            return;
        }
        PointCloudTPtr cloudOutTmp(new PointCloudT);

        PointT centerP{};
        this->compute3DCentroid(cloudIn, centerP);
        //bottomHighestPoint.push_back(centerP);
        vector<PointT> minPts;
        vector<double> dists;
        int num = ceil(360.0 / angleResolution);
        minPts.resize(num);
        dists.resize(num, -1.0);
        /*for (auto &d:dists) {
            cout<<d<<endl;
        }*/
        for (auto &p: *cloudIn) {
            double dist = this->dist2Point(p, centerP);
            double angleP = atan2(p.y - centerP.y, p.x - centerP.x);
            //cout << "angleP1 = " << angleP* 180 / M_PI << endl;
            if (angleP < 0) {
                angleP += M_PI * 2;
            }
            //cout << "angleP2 = " << angleP* 180 / M_PI << endl;
            int num_i = floor(angleP * 180 / M_PI / angleResolution);
            if (num_i >= num) {
                continue;
            }
            if (dists.at(num_i) < 0 || dists.at(num_i) > dist) {
                dists.at(num_i) = dist;
                minPts.at(num_i) = p;
            }
        }
        for (int i = 0; i < minPts.size(); ++i) {
            if (dists.at(i) > 0) {
                cloudOutTmp->push_back(minPts.at(i));
            }
        }
        *cloudOut = *cloudOutTmp;
    }

    unsigned int BaseDetection::compute3DCentroid(PointCloudTPtr cloudIn, PointT &centroid) {
        if (cloudIn->empty()) {
            return (0);
        }
        // Initialize to 0
        centroid = {};
        int cloudSize = cloudIn->size();
        float xSum = 0, ySum = 0, zSum = 0;
#pragma omp parallel for reduction(+:xSum, ySum, zSum)
        for (int i = 0; i < cloudSize; ++i) {
            PointT point = cloudIn->at(i);
            xSum = xSum + point.x;
            ySum = ySum + point.y;
            zSum = zSum + point.z;
        }
        //centroid /= static_cast<float >(cloudSize);
        centroid.x = xSum / static_cast<float >(cloudSize);
        centroid.y = ySum / static_cast<float >(cloudSize);
        centroid.z = zSum / static_cast<float >(cloudSize);
        return cloudSize;
    }

    void BaseDetection::getMaxMin(PointCloudTPtr _inputCloud, PointT &_minP, PointT &_maxP) {
        _minP = {};
        _maxP = {};
        if (_inputCloud->empty()) {
            //cout << "输入点云为空" << endl;
            return;
        }
        _minP = _inputCloud->at(0);
        _maxP = _inputCloud->at(0);
        for (PointT &p: *_inputCloud) {
            if (_minP.x > p.x) {
                _minP.x = p.x;
            }
            if (_minP.y > p.y) {
                _minP.y = p.y;
            }
            if (_minP.z > p.z) {
                _minP.z = p.z;
            }
            if (_maxP.x < p.x) {
                _maxP.x = p.x;
            }
            if (_maxP.y < p.y) {
                _maxP.y = p.y;
            }
            if (_maxP.z < p.z) {
                _maxP.z = p.z;
            }
        }
    }

    void BaseDetection::getZMaxMin(PointCloudTPtr _inputCloud, PointT &_zMinP, PointT &_zMaxP) {
        _zMinP = {};
        _zMaxP = {};
        if (_inputCloud->empty()) {
            //cout << "输入点云为空" << endl;
            return;
        }
        _zMinP = _inputCloud->at(0);
        _zMaxP = _inputCloud->at(0);
        for (PointT &p: *_inputCloud) {
            if (_zMinP.z > p.z) {
                _zMinP = p;
            }
            if (_zMaxP.z < p.z) {
                _zMaxP = p;
            }
        }
    }

    void BaseDetection::gridSegment(PointCloudTPtr _cloudIn, RectangleInfo _rectangleInfo, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec) {
        PointT p0 = _rectangleInfo.cornerPoints.at(0);
        PointT p1 = _rectangleInfo.cornerPoints.at(1);
        PointT p2 = _rectangleInfo.cornerPoints.at(2);
        PointT p3 = _rectangleInfo.cornerPoints.at(3);
        PointT directL{};
        PointT directW{};
        directL.x = (p1.x - p0.x) / _rectangleInfo.l;
        directL.y = (p1.y - p0.y) / _rectangleInfo.l;
        directL.z = (p1.z - p0.z) / _rectangleInfo.l;
        directW.x = (p2.x - p1.x) / _rectangleInfo.w;
        directW.y = (p2.y - p1.y) / _rectangleInfo.w;
        directW.z = (p2.z - p1.z) / _rectangleInfo.w;

        double gridL = _rectangleInfo.l / _numL;
        double gridW = _rectangleInfo.w / _numW;
        for (int i = 0; i < _numL; ++i) {
            for (int j = 0; j < _numW; ++j) {
                RectangleGrid rectangleGrid{};
                PointT gp0{};
                gp0.x = p0.x + directL.x * gridL * i + directW.x * gridW * j;
                gp0.y = p0.y + directL.y * gridL * i + directW.y * gridW * j;
                gp0.z = p0.z + directL.z * gridL * i + directW.z * gridW * j;
                PointT gp1{};
                gp1.x = gp0.x + directL.x * gridL;
                gp1.y = gp0.y + directL.y * gridL;
                gp1.z = gp0.z + directL.z * gridL;
                PointT gp2{};
                gp2.x = gp1.x + directW.x * gridW;
                gp2.y = gp1.y + directW.y * gridW;
                gp2.z = gp1.z + directW.z * gridW;
                PointT gp3{};
                gp3.x = gp0.x + directW.x * gridW;
                gp3.y = gp0.y + directW.y * gridW;
                gp3.z = gp0.z + directW.z * gridW;

                PointT gpCenter{};
                gpCenter.x = (gp0.x + gp2.x) / 2.0;
                gpCenter.y = (gp0.y + gp2.y) / 2.0;
                gpCenter.z = (gp0.z + gp2.z) / 2.0;

                rectangleGrid.cornerPoints.push_back(gp0);
                rectangleGrid.cornerPoints.push_back(gp1);
                rectangleGrid.cornerPoints.push_back(gp2);
                rectangleGrid.cornerPoints.push_back(gp3);
                rectangleGrid.l = gridL;
                rectangleGrid.w = gridW;
                rectangleGrid.centerPoint = gpCenter;

                std::vector<cv::Point2d> polygon;
                for (int k = 0; k < 4; ++k) {
                    cv::Point2d point2D(rectangleGrid.cornerPoints.at(k).x, rectangleGrid.cornerPoints.at(k).y);
                    polygon.push_back(point2D);
                }

                PointCloudTPtr cloudROI(new PointCloudT);
                roifilter::ROIFilter roiFilter;
                //double cellSize = rectangleGrid.w * 0.3 < 0.2 ? rectangleGrid.w * 0.3 : 0.2;
                roiFilter.roiFilter(_cloudIn, polygon, 0.2, cloudROI);

                PointT minP{};
                PointT maxP{};
                this->getZMaxMin(cloudROI, minP, maxP);
                rectangleGrid.highestPoint = maxP;
                rectangleGrid.lowestPoint = minP;

                _rectangleGridVec.push_back(rectangleGrid);
            }
        }
    }

    void
    BaseDetection::gridSegment(PointCloudTPtr _cloudIn, std::vector<PointT> _rectangleInfo, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec) {
        PointT p0 = _rectangleInfo.at(0);
        PointT p1 = _rectangleInfo.at(1);
        PointT p2 = _rectangleInfo.at(2);
        PointT p3 = _rectangleInfo.at(3);
        double l = sqrt((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y));
        double w = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        PointT directL{};
        PointT directW{};
        directL.x = (p1.x - p0.x) / l;
        directL.y = (p1.y - p0.y) / l;
        directL.z = (p1.z - p0.z) / l;
        directW.x = (p2.x - p1.x) / w;
        directW.y = (p2.y - p1.y) / w;
        directW.z = (p2.z - p1.z) / w;

        double gridL = l / _numL;
        double gridW = w / _numW;
        for (int i = 0; i < _numL; ++i) {
            for (int j = 0; j < _numW; ++j) {
                RectangleGrid rectangleGrid{};
                PointT gp0{};
                gp0.x = p0.x + directL.x * gridL * i + directW.x * gridW * j;
                gp0.y = p0.y + directL.y * gridL * i + directW.y * gridW * j;
                gp0.z = p0.z + directL.z * gridL * i + directW.z * gridW * j;
                PointT gp1{};
                gp1.x = gp0.x + directL.x * gridL;
                gp1.y = gp0.y + directL.y * gridL;
                gp1.z = gp0.z + directL.z * gridL;
                PointT gp2{};
                gp2.x = gp1.x + directW.x * gridW;
                gp2.y = gp1.y + directW.y * gridW;
                gp2.z = gp1.z + directW.z * gridW;
                PointT gp3{};
                gp3.x = gp0.x + directW.x * gridW;
                gp3.y = gp0.y + directW.y * gridW;
                gp3.z = gp0.z + directW.z * gridW;

                PointT gpCenter{};
                gpCenter.x = (gp0.x + gp2.x) / 2.0;
                gpCenter.y = (gp0.y + gp2.y) / 2.0;
                gpCenter.z = (gp0.z + gp2.z) / 2.0;

                rectangleGrid.cornerPoints.push_back(gp0);
                rectangleGrid.cornerPoints.push_back(gp1);
                rectangleGrid.cornerPoints.push_back(gp2);
                rectangleGrid.cornerPoints.push_back(gp3);
                rectangleGrid.l = gridL;
                rectangleGrid.w = gridW;
                rectangleGrid.centerPoint = gpCenter;

                std::vector<cv::Point2d> polygon;
                for (int k = 0; k < 4; ++k) {
                    cv::Point2d point2D(rectangleGrid.cornerPoints.at(k).x, rectangleGrid.cornerPoints.at(k).y);
                    polygon.push_back(point2D);
                }

                PointCloudTPtr cloudROI(new PointCloudT);
                roifilter::ROIFilter roiFilter;
                roiFilter.roiFilter(_cloudIn, polygon, 0.2, cloudROI);

                PointT minP{};
                PointT maxP{};
                this->getZMaxMin(cloudROI, minP, maxP);
                rectangleGrid.highestPoint = maxP;
                rectangleGrid.lowestPoint = minP;

                _rectangleGridVec.push_back(rectangleGrid);
            }
        }
    }

    void BaseDetection::gridSegment(PointCloudTPtr _cloudIn, std::vector<cv::Point2d> polygon, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec) {
        cv::Point2d p0 = polygon.at(0);
        cv::Point2d p1 = polygon.at(1);
        cv::Point2d p2 = polygon.at(2);
        cv::Point2d p3 = polygon.at(3);
        double l = sqrt((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y));
        double w = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        PointT directL{};
        PointT directW{};
        directL.x = (p1.x - p0.x) / l;
        directL.y = (p1.y - p0.y) / l;
        directL.z = 0;
        directW.x = (p2.x - p1.x) / w;
        directW.y = (p2.y - p1.y) / w;
        directW.z = 0;

        double gridL = l / _numL;
        double gridW = w / _numW;
        for (int i = 0; i < _numL; ++i) {
            for (int j = 0; j < _numW; ++j) {
                RectangleGrid rectangleGrid{};
                PointT gp0{};
                gp0.x = p0.x + directL.x * gridL * i + directW.x * gridW * j;
                gp0.y = p0.y + directL.y * gridL * i + directW.y * gridW * j;
                gp0.z = 0;
                PointT gp1{};
                gp1.x = gp0.x + directL.x * gridL;
                gp1.y = gp0.y + directL.y * gridL;
                gp1.z = gp0.z + directL.z * gridL;
                PointT gp2{};
                gp2.x = gp1.x + directW.x * gridW;
                gp2.y = gp1.y + directW.y * gridW;
                gp2.z = gp1.z + directW.z * gridW;
                PointT gp3{};
                gp3.x = gp0.x + directW.x * gridW;
                gp3.y = gp0.y + directW.y * gridW;
                gp3.z = gp0.z + directW.z * gridW;

                PointT gpCenter{};
                gpCenter.x = (gp0.x + gp2.x) / 2.0;
                gpCenter.y = (gp0.y + gp2.y) / 2.0;
                gpCenter.z = (gp0.z + gp2.z) / 2.0;

                rectangleGrid.cornerPoints.push_back(gp0);
                rectangleGrid.cornerPoints.push_back(gp1);
                rectangleGrid.cornerPoints.push_back(gp2);
                rectangleGrid.cornerPoints.push_back(gp3);
                rectangleGrid.l = gridL;
                rectangleGrid.w = gridW;
                rectangleGrid.centerPoint = gpCenter;

                std::vector<cv::Point2d> polygon;
                for (int k = 0; k < 4; ++k) {
                    cv::Point2d point2D(rectangleGrid.cornerPoints.at(k).x, rectangleGrid.cornerPoints.at(k).y);
                    polygon.push_back(point2D);
                }

                PointCloudTPtr cloudROI(new PointCloudT);
                roifilter::ROIFilter roiFilter;
                roiFilter.roiFilter(_cloudIn, polygon, 0.2, cloudROI);

                PointT minP{};
                PointT maxP{};
                this->getZMaxMin(cloudROI, minP, maxP);
                rectangleGrid.highestPoint = maxP;
                rectangleGrid.lowestPoint = minP;

                _rectangleGridVec.push_back(rectangleGrid);
            }
        }
    }

    void BaseDetection::gridSegment(PointCloudTPtr _cloudIn, std::string rectangleROIFile, int _numL, int _numW, vector<RectangleGrid> &_rectangleGridVec) {
        roifilter::ROIFilter roiFilter;
        std::vector<cv::Point2d> polygon;
        roiFilter.readRegionPoints(rectangleROIFile, polygon);
        this->gridSegment(_cloudIn, polygon, _numL, _numW, _rectangleGridVec);
    }
} // AutomatedPort