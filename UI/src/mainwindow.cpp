#include "mainwindow.h"

MainWindow::MainWindow(PTZ* ptz_,LidarDriveInterface* lidarDriveInterface_,QWidget *parent)
    : QMainWindow(parent),ptz(ptz_),lidarDriveInterface(lidarDriveInterface_)
{

    setWindowTitle(tr("PortMonitor"));

    main_title_bar =  new MainTitleBar();
    //    vtkNativeWindow = new QVTKOpenGLNativeWindow();
    //    echartWindow = new EchartWindow();
    imageWidget = new ImageWidget();
    mainImageWidget = new ImageWidget();
    add_lidar = new AddLidar();
    QObject::connect(add_lidar,SIGNAL(SendSet(TotalParams)), this, SLOT(params_set(TotalParams)));

    paint_area = new PaintArea();
    setROI = new SetROI();

    paint_area->scroll = setROI->scrollarea;
    setROI->scrollarea->takeWidget();
    setROI->scrollarea->setWidget(paint_area);

    diary= new Diary();
    alarm = new Alarm();

    camera_viewer =new CameraViewer();
    web_ui = new WebUi();



    initPointCShow();

    initMenu();
    initToolBar();
    initConnect();
    mainLayOut();
    mainEventCallback();




    this->setWindowFlags(Qt::FramelessWindowHint);
    this->main_title_bar->setParentWidget(this);
    //    this->main_title_bar->setStyleSheet("border-image: url(:/image/title.png);");
    //    this->main_title_bar->layoutDirection();
    //    QFrame#frameBorder{border-image: url(:/images/title.png);}
    this->setWindowIcon(QIcon(":/image/leishen.ico"));
}



void MainWindow::mainEventCallback()
{
    my_timer = new QTimer(this);
    my_timer->start(50);
    connect(my_timer,&QTimer::timeout,[=](){
        if(ptz != nullptr )
        {

        }
        if(lidarDriveInterface != nullptr)
        {
            updatePointCould();
        }
    });
}


void MainWindow::updatePointCould()
{
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if(!this->lidarDriveInterface->get_lidar_data(ptr))
    {
        return;
    }

    std::string name ="All_cloud";
    this->viewer->removePointCloud(name);
    this->viewer->addPointCloud(ptr,name);
    this->viewer->updatePointCloud(ptr,name);

    this->viewer->getRenderWindow()->GetInteractor()->Render();
    qvtkOpenglNativeWidget->update();

}


void MainWindow::initPointCShow()
{
    qvtkOpenglNativeWidget = new QVTKOpenGLNativeWidget();

    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    this->viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "All_cloud", false));

    qvtkOpenglNativeWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(qvtkOpenglNativeWidget->GetInteractor(), qvtkOpenglNativeWidget->GetRenderWindow());
    qvtkOpenglNativeWidget->update();

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();



    //    *********************************************test*********************************************/
    //*打开点云文件
    //    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ljy/xmh_huanwei.pcd",*cloud)==-1)
    //    {
    //        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
    //        return;
    //    }
    //    // 显示结果图
    //    viewer->setBackgroundColor (0, 0, 0); //设置背景
    //    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
    //    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
    //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //    viewer->resetCamera ();
    //    update ();
    //    /**********************************************test********************************************/
}

//void MainWindow::initCameraShow()
//{
//    //摄像头

//}


MainWindow::~MainWindow()
{

}

void MainWindow::initParamsEvent(std::shared_ptr<ParamsEvent<ObserverFunc>> params_event_)
{
    params_event = params_event_;

    std::lock_guard<std::mutex>(params_event->getMylock());

//    params_event->getTotalParams().lidarBase.DataAddress = "dasdasd";
//    params_event->Notify(params_event->getTotalParams());
}

void MainWindow::globalParamsCallback(TotalParams msg,bool save_flag)
{
    this->add_lidar->ShowData(msg);
}


void MainWindow::params_set(TotalParams msg)
{
    //msg.printData();
    std::lock_guard<std::mutex>(params_event->getMylock());
    msg.protocol_data = params_event->getTotalParams().protocol_data;
    params_event->getTotalParams() = msg;
    params_event->Notify(params_event->getTotalParams(),false);
    params_event->saveData();
}

void MainWindow::mainLayOut()
{

    this->mainLayout = new QVBoxLayout();
    this->headLayout = new QVBoxLayout();
    this->body_frame_layout = new QVBoxLayout();
    this->body_layout = new QHBoxLayout();
    this->left_body_layout = new QVBoxLayout();
    this->right_body_layout = new QVBoxLayout();

    this->right_camera_layout = new QHBoxLayout();
    this->right_data_layout = new QHBoxLayout();
    this->right_table_layout = new QHBoxLayout();

    this->vtkLayout = new QVBoxLayout();

    this->right_camera_widget = new QWidget(this);
    this->right_data_widget = new QWidget(this);
    this->right_table_widget = new QWidget(this);

    mainLayout->addLayout(headLayout);
    mainLayout->addLayout(body_frame_layout);


    body_frame_layout->addWidget(mainImageWidget);
    mainImageWidget->setLayout(body_layout);


    headLayout->addWidget(main_title_bar);

    body_layout->addLayout(left_body_layout);
    body_layout->addLayout(right_body_layout);
    body_layout->setStretchFactor(left_body_layout,2);
    body_layout->setStretchFactor(right_body_layout,1);

    static Diary* cur_1 = new Diary();
    static Diary* cur_2 = new Diary();
    static Diary* cur_3 = new Diary();

    right_camera_layout->addWidget(camera_viewer);

    //right_data_layout->addItem()
    right_data_layout->addWidget(diary);
    right_data_layout->addWidget(alarm);
    right_data_layout->setStretchFactor(diary,1);
    right_data_layout->setStretchFactor(alarm,1);

    //right_body_layout->addWidget(cv4Demo);
    right_body_layout->addLayout(right_camera_layout);
    right_body_layout->addLayout(right_data_layout);
    right_body_layout->addLayout(right_table_layout);
    right_body_layout->setStretchFactor(right_camera_layout,1);
    right_body_layout->setStretchFactor(right_data_layout,1);
    right_body_layout->setStretchFactor(right_table_layout,1);


    right_table_layout->addWidget(web_ui);

    imageWidget->setLayout(vtkLayout);
    vtkLayout->addWidget(qvtkOpenglNativeWidget);
    left_body_layout->addWidget(imageWidget);

    this->main_widget = new QWidget(this);
    main_widget->setLayout(mainLayout);
    this->setCentralWidget(main_widget);

}


void MainWindow::initMenu()
{
    //QMenuBar *menuBar = this->menuBar();
    //add three menus
    fileMenu = new QMenu(tr("&File"),this);
    viewMenu = new QMenu(tr("&View"),this);
    helpMenu = new QMenu(tr("&About"),this);

    //add actions and add it into corresponding menu
    view_mode = new QAction(tr("&Mode"),this);
    view_mode -> setShortcut(tr("ctrl+O"));
    view1_mode = new QAction(tr("&Mode1"),this);
    view1_mode -> setShortcut(tr("ctrl+P"));
    view2_mode = new QAction(tr("&Mode2"),this);
    view2_mode -> setShortcut(tr("ctrl+Q"));

    param_set = new QAction(tr("&Params"),this);
    lidar_area_set = new QAction(tr("&Areas"),this);

    connect(view_mode, SIGNAL(triggered(bool)), this, SLOT(view_mode_Action()));
    connect(view1_mode, SIGNAL(triggered(bool)), this, SLOT(view_mode1_Action()));
    connect(view2_mode, SIGNAL(triggered(bool)), this, SLOT(view_mode2_Action()));
    connect(param_set, SIGNAL(triggered(bool)), this, SLOT(param_set_Action()));
    connect(lidar_area_set, SIGNAL(triggered(bool)), this, SLOT(lidar_area_set_Action()));

}

void MainWindow::initToolBar()
{
    //add a toolbar and add its actions
    fileToolBar = new QToolBar(this);

    fileToolBar -> addAction(view_mode);
    fileToolBar -> addAction(view1_mode);
    fileToolBar -> addAction(view2_mode);

    fileToolBar ->addSeparator();

    fileToolBar -> addAction(param_set);
    fileToolBar -> addAction(lidar_area_set);


    //addToolBar(Qt::LeftToolBarArea,fileToolBar);
    addToolBar(Qt::BottomToolBarArea,fileToolBar);
    //    addToolBarBreak(Qt::BottomToolBarArea);
    //    addToolBar(Qt::BottomToolBarArea, toolBar);
}

void MainWindow::initConnect()
{
    //singals and slots
    //    connect(openAct,SIGNAL(triggered()),this,SLOT(open()));
    //    connect(printAct,SIGNAL(triggered()),this,SLOT(print()));
    //    connect(exitAct,SIGNAL(triggered()),this,SLOT(exit()));
    //    connect(zoomInAct,SIGNAL(triggered()),this,SLOT(zoomIn()));
    //    connect(zoomOutAct,SIGNAL(triggered()),this,SLOT(zoomOut()));
    //    connect(normalSizeAct,SIGNAL(triggered()),this,SLOT(normalSize()));
    //    connect(fitToWindowAct,SIGNAL(triggered()),this,SLOT(fitToWindow()));
    //    connect(aboutAct,SIGNAL(triggered()),this,SLOT(about()));
    //    connect(aboutQtAct,SIGNAL(triggered()),this,SLOT(aboutQt()));
}

void MainWindow::createActions()
{

}

void MainWindow::param_set_Action()
{
    this->add_lidar->show();
}

void MainWindow::lidar_area_set_Action()
{
    this->setROI->show();
    this->setROI->showMaximized();
}


void MainWindow::view_mode_Action()
{
    //    cv4Demo->hide();
    imageWidget->setVisible(true);
    qvtkOpenglNativeWidget->setVisible(true);
    for (int i = 0; i < right_body_layout->count(); ++i) {
        for (int y = 0; y < right_body_layout->itemAt(i)->layout()->count(); ++y)
        {
            auto z = right_body_layout->itemAt(i)->layout()->itemAt(y)->widget();
            if (z != NULL)
                z->setVisible(true);
        }
    }
}

void MainWindow::view_mode1_Action()
{
    //    cv4Demo->hide();
    imageWidget->setVisible(true);
    qvtkOpenglNativeWidget->setVisible(true);
    for (int i = 0; i < right_body_layout->count(); ++i) {
        for (int y = 0; y < right_body_layout->itemAt(i)->layout()->count(); ++y)
        {
            auto z = right_body_layout->itemAt(i)->layout()->itemAt(y)->widget();
            if (z != NULL)
                z->setVisible(false);
        }
    }

}

void MainWindow::view_mode2_Action()
{
    //        cv4Demo1->setVisible(false);
    //        cv4Demo2->setVisible(false);
    //        imageWidget->setVisible(false);
    //        vtkNativeWindow->setVisible(false);
    //        this->update();
}


CameraViewer* MainWindow::getCameraWidget()
{
    return this->camera_viewer;
}

















//implement slot functions
//void MainWindow::open()
//{

//}

//void MainWindow::print()
//{

//}

//void MainWindow::exit()
//{

//}

//void MainWindow::zoomIn()
//{

//}

//void MainWindow::zoomOut()
//{

//}

//void MainWindow::normalSize()
//{

//}

//void MainWindow::fitToWindow()
//{

//}

//void MainWindow::about()
//{

//}

//void MainWindow::aboutQt()
//{

//}



