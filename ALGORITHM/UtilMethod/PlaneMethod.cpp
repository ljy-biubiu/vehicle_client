//
// Created by niujy on 2022/11/16.
//


#include "PlaneMethod.h"

namespace planemethod {
    PlaneMethod::PlaneMethod() {
        normalLength = 1.0;
    }

    PlaneMethod::~PlaneMethod() {

    }

    void PlaneMethod::calcPlane(PointCloudTPtr inputCloud, double threshold, PlaneParams &planeParams) {
        planeParams = {};
        cout << "calcPlane inputCloud : " << inputCloud->size() << endl;

        pcl::SACSegmentation<PointT> seg;    //分割对象
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        //planeParams.coefficients = static_cast<boost::shared_ptr<pcl::ModelCoefficients>>(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setOptimizeCoefficients(true);           //设置对估计模型优化
        seg.setModelType(pcl::SACMODEL_PLANE);//设置分割模型为平面
        seg.setMethodType(pcl::SAC_RANSAC);          //参数估计方法
        seg.setDistanceThreshold(threshold);         //设置内点到模型的距离允许最大值
        seg.setMaxIterations(inputCloud->size());
        //seg.setAxis(Eigen::Vector3f(0, 0, 1));
        //seg.setEpsAngle(30.0f * (M_PI / 180.0f));
        seg.setInputCloud(inputCloud);
        seg.segment(*inliers, *coefficients);

        *planeParams.coefficients = *coefficients;

        planeParams.A = coefficients->values[0];
        planeParams.B = coefficients->values[1];
        planeParams.C = coefficients->values[2];
        planeParams.D = coefficients->values[3];

        PointCloudTPtr filterPlaneCloud(new PointCloudT);
        PointCloudTPtr filterNonPlaneCloud(new PointCloudT);
        pcl::ExtractIndices<PointT> extract;   //点提取对象
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);//设置成true是保存滤波后剩余的点，false是保存在区域内的点
        extract.filter(*filterPlaneCloud);
        extract.setNegative(true);//设置成true是保存滤波后剩余的点，false是保存在区域内的点
        extract.filter(*filterNonPlaneCloud);

        *planeParams.planeCloud = *filterPlaneCloud;
        *planeParams.nonPlaneCloud = *filterNonPlaneCloud;

        // 计算重心
        Eigen::Vector4d centreOfGravity = Eigen::Vector4d::Zero();
        pcl::compute3DCentroid(*filterPlaneCloud, centreOfGravity);
        planeParams.centreOfGravity.x = centreOfGravity[0];
        planeParams.centreOfGravity.y = centreOfGravity[1];
        planeParams.centreOfGravity.z = centreOfGravity[2];

        // 判断法线方向，使法线指向坐标原点
        /*double n_center = planeParams.A * planeParams.centreOfGravity.x + planeParams.B * planeParams.centreOfGravity.y +
                          planeParams.C * planeParams.centreOfGravity.z;*/
        if (planeParams.C < 0) {
            planeParams.A *= -1.0;
            planeParams.B *= -1.0;
            planeParams.C *= -1.0;
            planeParams.D *= -1.0;
            planeParams.coefficients->values.at(0) *= -1.0;
            planeParams.coefficients->values.at(1) *= -1.0;
            planeParams.coefficients->values.at(2) *= -1.0;
            planeParams.coefficients->values.at(3) *= -1.0;
        }

        planeParams.arrowPoint.x = planeParams.centreOfGravity.x + planeParams.A * normalLength;
        planeParams.arrowPoint.y = planeParams.centreOfGravity.y + planeParams.B * normalLength;
        planeParams.arrowPoint.z = planeParams.centreOfGravity.z + planeParams.C * normalLength;

        this->printPlaneParams(planeParams);
    }

    void PlaneMethod::calcNormalPlane(PointCloudTPtr inputCloud, double threshold, Eigen::Vector3f axis, PlaneParams &planeParams) {
        planeParams = {};
//        cout << "calcPlane inputCloud : " << inputCloud->size() << endl;

        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;  //法线估计对象
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;    //分割对象
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
        ne.setSearchMethod(tree);
        ne.setInputCloud(inputCloud);
        ne.setRadiusSearch(2);
        //ne.setKSearch(50);
        ne.setNumberOfThreads(30);
        ne.compute(*cloudNormals);

        seg.setOptimizeCoefficients(true);           //设置对估计模型优化
        seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);//设置分割模型为平面
        seg.setMethodType(pcl::SAC_RANSAC);          //参数估计方法
        seg.setDistanceThreshold(threshold);         //设置内点到模型的距离允许最大值
        seg.setMaxIterations(inputCloud->size());
        seg.setAxis(axis);
        seg.setEpsAngle(20.0f * (M_PI / 180.0f));
        seg.setInputCloud(inputCloud);
        seg.setNormalDistanceWeight(0.1);
        seg.setInputNormals(cloudNormals);
        seg.segment(*inliers, *coefficients);

        *planeParams.coefficients = *coefficients;

        planeParams.A = coefficients->values[0];
        planeParams.B = coefficients->values[1];
        planeParams.C = coefficients->values[2];
        planeParams.D = coefficients->values[3];

        PointCloudTPtr filterPlaneCloud(new PointCloudT);
        PointCloudTPtr filterNonPlaneCloud(new PointCloudT);
        pcl::ExtractIndices<PointT> extract;   //点提取对象
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);//设置成true是保存滤波后剩余的点，false是保存在区域内的点
        extract.filter(*filterPlaneCloud);
        extract.setNegative(true);//设置成true是保存滤波后剩余的点，false是保存在区域内的点
        extract.filter(*filterNonPlaneCloud);

        *planeParams.planeCloud = *filterPlaneCloud;
        *planeParams.nonPlaneCloud = *filterNonPlaneCloud;

        // 计算重心
        Eigen::Vector4d centreOfGravity = Eigen::Vector4d::Zero();
        pcl::compute3DCentroid(*filterPlaneCloud, centreOfGravity);
        planeParams.centreOfGravity.x = centreOfGravity[0];
        planeParams.centreOfGravity.y = centreOfGravity[1];
        planeParams.centreOfGravity.z = centreOfGravity[2];

        // 判断法线方向，使法线指向坐标原点
        /*double n_center = planeParams.A * planeParams.centreOfGravity.x + planeParams.B * planeParams.centreOfGravity.y +
                          planeParams.C * planeParams.centreOfGravity.z;*/
        if (planeParams.C < 0) {
            planeParams.A *= -1.0;
            planeParams.B *= -1.0;
            planeParams.C *= -1.0;
            planeParams.D *= -1.0;
            planeParams.coefficients->values.at(0) *= -1.0;
            planeParams.coefficients->values.at(1) *= -1.0;
            planeParams.coefficients->values.at(2) *= -1.0;
            planeParams.coefficients->values.at(3) *= -1.0;
        }

        planeParams.arrowPoint.x = planeParams.centreOfGravity.x + planeParams.A * normalLength;
        planeParams.arrowPoint.y = planeParams.centreOfGravity.y + planeParams.B * normalLength;
        planeParams.arrowPoint.z = planeParams.centreOfGravity.z + planeParams.C * normalLength;

//        this->printPlaneParams(planeParams);
    }

    void PlaneMethod::calcRotationBy2Plane(PlaneParams planeBefore, PlaneParams planeAfter, Eigen::Matrix4d &rotTrans, Eigen::Vector4d &projectionPoint) {
        Eigen::Matrix3d rotMatrix = Eigen::Matrix3d::Identity();
        Eigen::Vector3d vectorBefore(planeBefore.A, planeBefore.B, planeBefore.C);
        Eigen::Vector3d vectorAfter(planeAfter.A, planeAfter.B, planeAfter.C);

        rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();

        Eigen::Vector3d centerPoint(planeBefore.centreOfGravity.x, planeBefore.centreOfGravity.y,
                                    planeBefore.centreOfGravity.z);
        //Eigen::Vector3d centerPointRot(0, 0, 0);
        //centerPointRot = rotMatrix * centerPoint;
        double A = planeAfter.A;
        double B = planeAfter.B;
        double C = planeAfter.C;
        double D = planeAfter.D;
        // 投影点
        double xp = ((B * B + C * C) * centerPoint[0] - A * (B * centerPoint[1] + C * centerPoint[2] + D))
                    / (A * A + B * B + C * C);
        double yp = ((A * A + C * C) * centerPoint[1] - B * (A * centerPoint[0] + C * centerPoint[2] + D))
                    / (A * A + B * B + C * C);
        double zp = ((A * A + B * B) * centerPoint[2] - C * (A * centerPoint[0] + B * centerPoint[1] + D))
                    / (A * A + B * B + C * C);
        projectionPoint[0] = xp;
        projectionPoint[1] = yp;
        projectionPoint[2] = zp;
        projectionPoint[3] = 1.0;
        //cout<<"projectionPoint : "<<xp<<" , "<<yp<<" , "<<zp<<endl;

        Eigen::Vector3d trans1(-centerPoint[0] + xp, -centerPoint[1] + yp, -centerPoint[2] + zp);
        Eigen::Matrix4d rotTrans0 = Eigen::Matrix4d::Identity();
        rotTrans0.block<3, 3>(0, 0) = rotMatrix;
        Eigen::Matrix4d rotTrans1 = Eigen::Matrix4d::Identity();
        rotTrans1.block<3, 1>(0, 3) = trans1;
        //rotTrans1 << rotMatrix, trans1, 0, 0, 0, 1;

        Eigen::Matrix4d rotTrans2 = Eigen::Matrix4d::Identity();
        rotTrans2(0, 3) = centerPoint[0];
        rotTrans2(1, 3) = centerPoint[1];
        rotTrans2(2, 3) = centerPoint[2];

        Eigen::Matrix4d rotTrans3 = Eigen::Matrix4d::Identity();
        rotTrans3(0, 3) = -centerPoint[0];
        rotTrans3(1, 3) = -centerPoint[1];
        rotTrans3(2, 3) = -centerPoint[2];

        rotTrans = rotTrans2 * rotTrans1 * rotTrans0 * rotTrans3;

        //Eigen::Vector3d trans(-centerPoint[0] + xp, -centerPoint[1] + yp, -centerPoint[2] + zp);
        //rotTrans << rotMatrix, trans, 0, 0, 0, 1;
        // cout<<trans.transpose()<<endl;
    }

    void PlaneMethod::printPlaneParams(PlaneParams planeParams) {
        cout << "planeParams : "
             << planeParams.A << " , "
             << planeParams.B << " , "
             << planeParams.C << " , "
             << planeParams.D << " , "
             << planeParams.centreOfGravity.x << " , "
             << planeParams.centreOfGravity.y << " , "
             << planeParams.centreOfGravity.z
             << endl;
    }


} // planemethod