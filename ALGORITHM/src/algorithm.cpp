#include "algorithm.h"

Algorithm::Algorithm()
{
    //groundMaterialDetect();
    //hatchDetectClouds();
}

void Algorithm::init()
{

}

void Algorithm::run()
{
    std::thread t1(std::bind(&Algorithm::groundMaterialDetect, this));
    t1.detach();

    std::thread t2(std::bind(&Algorithm::hatchDetectClouds, this));
    t2.detach();
}

double Algorithm::timespecMs() {
    timespec ts{};
    timespec_get(&ts, TIME_UTC);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / (1000.0 * 1000.0);
}

int Algorithm::getColorCloud(vector<PointCloudTPtr> cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorCloud, int colorStart, int colorEnd) {
    if (cloudIn.empty()) {
        return 0;
    }
    srand(time(nullptr));
    int totalPoint = 0;
    for (int i_seg = 0; i_seg < cloudIn.size(); i_seg++) {
        int clusters_size = cloudIn[i_seg]->size();
        int r = rand() % (colorEnd - colorStart) + colorStart;
        int g = rand() % (colorEnd - colorStart) + colorStart;
        int b = rand() % (colorEnd - colorStart) + colorStart;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int i_idx = 0; i_idx < clusters_size; i_idx++) {
            pcl::PointXYZRGB point;
            point.x = cloudIn[i_seg]->at(i_idx).x;
            point.y = cloudIn[i_seg]->at(i_idx).y;
            point.z = cloudIn[i_seg]->at(i_idx).z;
            point.r = r;
            point.g = g;
            point.b = b;
            tmpCloud->push_back(point);
        }
        *colorCloud += *tmpCloud;
        totalPoint += tmpCloud->size();
        //cout << "tmpCloud size = " << tmpCloud->size() << endl;
    }
    return totalPoint;
}

void Algorithm::show() {
    //可视化显示原始点云与边界提取结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("bianjie"));

    int v1(0);
    MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
    MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
    MView->setBackgroundColor(0.0, 0.0, 0.0, v2);
    MView->addText("Boudary point clouds", 10, 10, "v2_text", v2);
    MView->addCoordinateSystem(5.0);
    MView->initCameraParameters();
    MView->setCameraPosition(0, 0, 15, -9, 0, -19, 0, 1, 0);
    double st = timespecMs();
    while (running) {

        if (updateFlag) {
            updateFlag = false;
            PointCloudTPtr intersectionCloud(new PointCloudT);// 矩形角点点云
            vector<AutomatedPort::LineInfo> lineInfoVec;
            for (int i = 0; i < params_out.shipParamOut.rectangleInfoVec.size(); ++i) {
                lineInfoVec.push_back(params_out.shipParamOut.rectangleInfoVec.at(i).linesL.at(0));
                lineInfoVec.push_back(params_out.shipParamOut.rectangleInfoVec.at(i).linesL.at(1));
                lineInfoVec.push_back(params_out.shipParamOut.rectangleInfoVec.at(i).linesW.at(0));
                lineInfoVec.push_back(params_out.shipParamOut.rectangleInfoVec.at(i).linesW.at(1));
                for (int j = 0; j < 4; ++j) {
                    intersectionCloud->push_back(params_out.shipParamOut.rectangleInfoVec.at(i).cornerPoints.at(j));
                }
            }

            PointCloudTPtr bottomHighestCloud(new PointCloudT);// 底部最高点点云
            for (int i = 0; i < params_out.shipParamOut.bottomHighestPoint.size(); ++i) {
                bottomHighestCloud->push_back(params_out.shipParamOut.bottomHighestPoint.at(i));
            }

            //            cout << "time5 = " << timeMs() - st << " ms " << endl;
            //            st = timeMs();

            vector<PointCloudTPtr> allLineCloudVec;
            allLineCloudVec.clear();
            for (int i = 0; i < lineInfoVec.size(); ++i) {
                allLineCloudVec.push_back(lineInfoVec.at(i).cloud);
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            getColorCloud(allLineCloudVec, colorCloud, 50, 250);

            /*if (!colorCloud->empty() && !intersectionCloud->empty()) {
                pcl::PCDWriter writer;
                std::stringstream ss;
                ss << "boundary" << ".pcd";
                writer.write<pcl::PointXYZRGB>(ss.str(), *colorCloud, true);

                std::stringstream ss1;
                ss1 << "intersectionCloud" << ".pcd";
                writer.write<PointT>(ss1.str(), *intersectionCloud, true);
            }*/

            MView->removeAllPointClouds();
            MView->removeAllShapes();

            MView->addPointCloud<PointT>(params_out.shipParamOut.deckCloud, "sample cloud", v1);
            //MView->addPointCloud<PointT>(deckBottomCloud, "sample cloud1", v1);
            pcl::visualization::PointCloudColorHandlerGenericField<PointT> findColor(params_out.shipParamOut.deckBottomCloud, "z"); // 按照I字段进行渲染
            MView->addPointCloud<PointT>(params_out.shipParamOut.deckBottomCloud, findColor, "deckBottomCloud", v1);

//            //MView->addPointCloud<PointT>(boundaryCloud, "cloud_boundary", v1);
//            MView->addPointCloud(colorCloud, "cloud_boundary1", v2);
//            MView->addPointCloud<PointT>(intersectionCloud, "intersectionCloud", v2);
//            MView->addPointCloud<PointT>(bottomHighestCloud, "bottomHighestCloud", v1);
//            MView->addPointCloud<PointT>(bottomHighestCloud, "bottomHighestCloud2", v2);
//            MView->addPointCloud<PointT>(params_out.shipParamOut.polarCloud, "polarCloud", v2);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
//            //MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "sample cloud1", v1);
//            //MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v1);
//            //MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud_boundary1", v2);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "intersectionCloud", v2);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "intersectionCloud", v2);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "bottomHighestCloud", v1);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "bottomHighestCloud", v1);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "bottomHighestCloud2", v2);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "bottomHighestCloud2", v2);
//            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "polarCloud", v2);


//            for (int i = 0; i < params_out.shipParamOut.allClusters.size(); ++i) {
//                lshapedfit::ObjectInfo &_cluster = params_out.shipParamOut.allClusters.at(i);
//                Eigen::Vector3f translation(_cluster.x, _cluster.y, _cluster.z);
//                Eigen::Quaternionf rotation(cos(_cluster.heading / 2), 0, 0, sin(_cluster.heading / 2));//_cluster.heading * M_PI / 180.0
//                MView->addCube(translation, rotation, _cluster.l, _cluster.w, _cluster.h, to_string(i) + "cube", v2);
//                MView->setRepresentationToWireframeForAllActors();
//            }
//            for (int i = 0; i < lineInfoVec.size(); ++i) {
//                MView->addLine(*lineInfoVec.at(i).coefficients, "line" + to_string(i), v2);
//            }
//            for (int i = 0; i < params_out.shipParamOut.rectangleInfoVec.size(); ++i) {
//                MView->addText3D(Util::formatValue(params_out.shipParamOut.rectangleInfoVec.at(i).l, 2), params_out.shipParamOut.rectangleInfoVec.at(i).linesL.at(0).centerPoint, 1.0,
//                                 1.0, 1.0,
//                                 1.0, "line" + to_string(i), v2);
//                MView->addText3D(Util::formatValue(params_out.shipParamOut.rectangleInfoVec.at(i).w, 2), params_out.shipParamOut.rectangleInfoVec.at(i).linesW.at(0).centerPoint, 1.0,
//                                 1.0, 1.0,
//                                 1.0, "line1" + to_string(i), v2);
//            }
//            for (int i = 0; i < params_out.shipParamOut.bottomHighestPoint.size(); ++i) {
//                PointT p = params_out.shipParamOut.bottomHighestPoint.at(i);
//                string text = Util::formatValue(p.x, 2) + "," + Util::formatValue(p.y, 2) + "," + Util::formatValue(p.z, 2);
//                MView->addText3D(text, p, 1.0, 1.0, 1.0, 1.0, "point" + to_string(i), v2);
//            }

        }

        MView->spinOnce(1);
    }
}

void Algorithm::hatchDetectClouds() {
    double st = timespecMs();

//    std::thread t1(std::bind(&Algorithm::show, this));
//    t1.detach();

    string folderPath = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ship_cloud/";
    vector<string> vecStr;
    Util::scan_one_dir(folderPath.c_str(), vecStr);
    for (int m = 0; m < vecStr.size(); ++m) {
        if (m == vecStr.size() - 1) {
            m = 0;
        }
        //m = 9;
        cout << "******************************************" << endl;
        string path = folderPath + vecStr.at(m);
        PointCloudTPtr cloud_src(new PointCloudT);
        pcl::io::loadPCDFile(path, *cloud_src);

        cout << m << " , " << path << " , loadPCDFile = " << timespecMs() - st << " ms " << endl;
        st = timespecMs();

        // 舱口矩形检测
        AutomatedPort::ShipDetection shipDetection;
        AutomatedPort::ShipParamIn shipParamIn{};
        shipParamIn.rtParams = rtmethod::RTParams(-90, 0, 0, 0, 0, 0);//初始旋转平移参数
        shipParamIn.roiFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/ShipROIPoints.txt";
        shipParamIn.cloudSrc = cloud_src;//设置输入点云
        shipParamIn.gridNumY = process_params_in.shipParamIn.gridNumX;//3
        shipParamIn.gridNumX = process_params_in.shipParamIn.gridNumY;
        shipDetection.setShipParamIn(shipParamIn);
        shipDetection.runDetect();// 运行检测方法
        cout << "runDetect = " << timespecMs() - st << " ms " << endl;
        st = timespecMs();

        params_out.shipParamOut = shipDetection.getShipParamOut();

        std::lock_guard<std::mutex> lck(m_lock);

        recv_lidar_queue_.push(params_out);


        if (recv_lidar_queue_.size() > 3)
        {
            recv_lidar_queue_.pop();

            qWarning()<<"警告：接收到lidar驱动队列超过3！";
        }

        updateFlag = true;
    }
    running = false;
}

void Algorithm::showMaterial() {
    //可视化显示原始点云与边界提取结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Material"));

    //    int v1(0);
    //    MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    //    MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
    //    MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
    MView->setBackgroundColor(0.0, 0.0, 0.0, v2);
    MView->addText("Boudary point clouds", 10, 10, "v2_text", v2);
    MView->addCoordinateSystem(5.0);
    MView->initCameraParameters();
    MView->setCameraPosition(-8.75004, 29.7975, -43.4298, 5.77468, -44.1817, -43.746, 0, 0, 1);
    double st = timespecMs();
    while (running) {

        if (updateFlag) {
            updateFlag = false;

            PointCloudTPtr highestCloud(new PointCloudT);// 底部最高点点云
            for (int i = 0; i < params_out.groundParamOut.materialHighestPoint.size(); ++i) {
                highestCloud->push_back(params_out.groundParamOut.materialHighestPoint.at(i));
            }

            MView->removeAllPointClouds();
            MView->removeAllShapes();

            MView->addPointCloud<PointT>(params_out.groundParamOut.cloudSrcRT, "cloudSrcRT", v2);
            //MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloudSrcRT", v1);

            /*pcl::visualization::PointCloudColorHandlerGenericField<PointT> findColor1(cloudSrcRT, "z"); // 按照I字段进行渲染
            MView->addPointCloud<PointT>(cloudSrcRT, findColor1, "cloudSrcRT",v1);*/

            /*MView->addPointCloud<PointT>(groundCloud, "groundCloud", v2);
            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "groundCloud", v2);*/

            pcl::visualization::PointCloudColorHandlerGenericField<PointT> findColor(params_out.groundParamOut.groundCloud, "z"); // 按照I字段进行渲染
            MView->addPointCloud<PointT>(params_out.groundParamOut.groundCloud, findColor, "groundCloud", v2);

            MView->addPointCloud<PointT>(highestCloud, "highestCloud", v2);
            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "highestCloud", v2);
            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "highestCloud", v2);

            MView->addPointCloud<PointT>(highestCloud, "highestCloud1", v2);
            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "highestCloud1", v2);
            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "highestCloud1", v2);

            /*MView->addPointCloud<PointT>(groundPlaneParams.planeCloud, "groundPlaneParams", v2);
            MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "groundPlaneParams", v2);*/
            for (int i = 0; i < params_out.groundParamOut.materialHighestPoint.size(); ++i) {
                PointT p = params_out.groundParamOut.materialHighestPoint.at(i);
                string text = Util::formatValue(p.x, 2) + "," + Util::formatValue(p.y, 2) + "," + Util::formatValue(p.z, 2);
                MView->addText3D(text, p, 1.0, 1.0, 1.0, 1.0, "point" + to_string(i), v2);
                MView->addText3D(text, p, 1.0, 1.0, 1.0, 1.0, "point1" + to_string(i), v2);
            }
        }

        MView->spinOnce(1);
    }
}

void Algorithm::groundMaterialDetect() {
    double st = timespecMs();

//    std::thread t1(std::bind(&Algorithm::showMaterial, this));
//    t1.detach();

    string folderPath = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/material_cloud/";
    vector<string> vecStr;
    Util::scan_one_dir(folderPath.c_str(), vecStr);
    for (int m = 0; m < vecStr.size(); ++m) {
        if (m == vecStr.size() - 1) {
            m = 0;
        }
        cout << "******************************************" << endl;
        st = timespecMs();
        string path = folderPath + vecStr.at(m);
        PointCloudTPtr cloud_src(new PointCloudT);
        pcl::io::loadPCDFile(path, *cloud_src);

        cout << m << " , " << path << " , loadPCDFile = " << timespecMs() - st << " ms " << endl;
        st = timespecMs();

        AutomatedPort::GroundMaterialDetection materialDetection;
        AutomatedPort::GroundParamIn groundParamIn{};
        groundParamIn.rtParams = rtmethod::RTParams(-90, 0, 0, 0, 0, 0);//初始旋转平移参数
        groundParamIn.roiFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/GroundROIPoints.txt";
        groundParamIn.rectangleROIFile = "/home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor/ALGORITHM/ROIPointsFile/GroundRectanglePoints.txt";// 物料区划分N*M格子区域
        groundParamIn.cloudSrc = cloud_src;
        groundParamIn.gridNumX = process_params_in.groundParamIn.gridNumX;//2
        groundParamIn.gridNumY = process_params_in.groundParamIn.gridNumY;
        materialDetection.setGroundParamIn(groundParamIn);
        materialDetection.runDetect();

        cout << "materialDetection = " << timespecMs() - st << " ms " << endl;
        st = timespecMs();

        params_out.groundParamOut = materialDetection.getGroundParamOut();

        std::lock_guard<std::mutex> lck(m_lock);

        recv_lidar_queue_.push(params_out);


        if (recv_lidar_queue_.size() > 3)
        {
            recv_lidar_queue_.pop();

            qWarning()<<"警告：接收到lidar驱动队列超过3！";
        }

        updateFlag = true;
    }
    running = false;
}
