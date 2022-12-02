#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "maintitlebar.h"
#include "imageWidget.h"
#include "addlidar.h"
#include "paintarea.h"
#include "setroi.h"
#include "diary.h"
#include "alarm.h"
#include "cameraviewer.h"
#include "webui.h"


// Visualization Toolkit (VTK)
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include "vtkGenericOpenGLRenderWindow.h"

//pcl
#include <pcl/io/ifs_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>


#include <QLabel>
#include <QScrollArea>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QHBoxLayout>
#include <QWidget>
#include <QApplication>

#include "algorithm_interface.h"
#include "lidar_drive_interface.h"
#include "algorithm.h"
#include "ptz.h"
#include "paramsevent.hpp"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(PTZ* ptz_,LidarDriveInterface* lidarDriveInterface_,QApplication * my_app_,QWidget *parent = nullptr);
    ~MainWindow();

    CameraViewer * getCameraWidget();
    void updatePointCould();
    void initPTZ(PTZ*);
    void initAlgorithmInterface(AlgorithmInterface* msg)
    {
        algorithmInterface = msg;
    }
    void globalParamsCallback(TotalParams msg,bool save_flag);
    //    void globalParamsCallback(TotalParams msg);
    //    int globalParamsCallback(int msg);

private:
    void initMenu();
    void initToolBar();
    void initConnect();
    void createActions();
    void mainLayOut();
    void initPointCShow();
    void initObeject();

    void updateAlgPointCould();

    //void initCameraDriveShow();

    void mainEventCallback();

    QApplication * my_app;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    AddLidar *add_lidar;
    MainTitleBar *main_title_bar;
    //    EchartWindow *echartWindow;
    QVTKOpenGLNativeWidget *qvtkOpenglNativeWidget;
    ImageWidget *imageWidget;
    ImageWidget *mainImageWidget;
    PaintArea *paint_area;
    SetROI *setROI;
    Diary *diary;
    Alarm *alarm;
    CameraViewer *camera_viewer;
    WebUi *web_ui;

    PTZ* ptz;
    LidarDriveInterface* lidarDriveInterface;
    AlgorithmInterface* algorithmInterface = nullptr;

    QWidget *main_widget;
    QWidget *right_camera_widget;
    QWidget *right_data_widget;
    QWidget *right_table_widget;

    //layout
    QVBoxLayout *body_frame_layout;
    QVBoxLayout *mainLayout;
    QVBoxLayout *headLayout;
    QHBoxLayout *body_layout;
    QVBoxLayout *left_body_layout;
    QVBoxLayout *right_body_layout;
    QHBoxLayout *layout;

    QHBoxLayout *right_camera_layout;
    QHBoxLayout *right_data_layout;
    QHBoxLayout *right_table_layout;

    QVBoxLayout *vtkLayout;

    QLabel *imageLabel;
    QScrollArea *scrollAera;

    QMenu *fileMenu;
    QMenu *viewMenu;
    QMenu *helpMenu;

    QToolBar *fileToolBar;

    QAction *view_mode;
    QAction *view1_mode;
    QAction *view2_mode;
    QAction *param_set;
    QAction *lidar_area_set;

    QTimer *my_timer;

public:
    void initParamsEvent(std::shared_ptr<ParamsEvent<ObserverFunc>> params_event_);

private:
    std::shared_ptr<ParamsEvent<ObserverFunc>> params_event;


private slots:
    void view_mode_Action();
    void view_mode1_Action();
    void view_mode2_Action();

    void param_set_Action();
    void lidar_area_set_Action();

    void params_set(TotalParams msg);

};
#endif // MAINWINDOW_H


