#ifndef QVTKOPENGLWINDOW_H
#define QVTKOPENGLWINDOW_H

// Qt Library
#include <QDebug>
#include <QWidget>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

#include <dlfcn.h>
//using namespace cv;

typedef pcl::PointXYZRGBA       PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//点云数据
typedef struct vtkpointcloud{
    float x;
    float y;
    float z;
    unsigned int red;
    unsigned int green;
    unsigned int blue;
}VTK_POINT_CLOUD_S;

class QVTKOpenGLNativeWindow : public QVTKOpenGLNativeWidget
{
    Q_OBJECT
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;

    public:
    explicit QVTKOpenGLNativeWindow(QWidget *parent = 0);

    private:
        pcl::visualization::PCLVisualizer::Ptr viewer;
        //PointCloudT::Ptr                       cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

#endif // QVTKOPENGLWINDOW_H
