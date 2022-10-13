#include "qvtkopenglwindow.h"

#include <qpainter.h>
#include <qdebug.h>
#include <pcl/common/io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include "vtkGenericOpenGLRenderWindow.h"


QVTKOpenGLNativeWindow::QVTKOpenGLNativeWindow(QWidget *parent)
    : QVTKOpenGLNativeWidget(parent)
{
    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    this->SetRenderWindow(viewer->getRenderWindow());                       //将数据转至pclviewer 数据 转到vtk来渲染
    viewer->setupInteractor(this->GetInteractor(),this->GetRenderWindow());


    //*打开点云文件
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ljy/xmh_huanwei.pcd",*cloud)==-1)
    {
      PCL_ERROR("Couldn't read file test_pcd.pcd\n");
      return;
    }

    // 显示结果图
    viewer->setBackgroundColor (0, 0, 0); //设置背景
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->resetCamera ();
    update ();
}

