#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    setWindowTitle(tr("cloud viewer demo"));

//    widget = new QWidget(this);

//    layout = new QHBoxLayout();
//    layout->addWidget(vtkNativeWindow);
////    layout->addSpacing(50);
////    layout->addWidget(cv4Demo);

////    cv4Demo->setMaximumSize(500, 500);
////    cv4Demo->setMinimumSize(500, 500);
////    vtkNativeWindow->setMaximumSize(500, 500);
////    vtkNativeWindow->setMinimumSize(500, 500);

//    this->widget->setLayout(layout);
//    this->setCentralWidget(widget);

    cv4Demo = new Cv4Demo();
    cv4Demo1 = new Cv4Demo();
    cv4Demo2 = new Cv4Demo();
    vtkNativeWindow = new QVTKOpenGLNativeWindow();
    echartWindow = new EchartWindow();

    //设置背景黑色
    QPalette palBackGround(this->palette());
    palBackGround.setColor(QPalette::Background, QColor(74, 74, 74));
    this->setAutoFillBackground(true);
    this->setPalette(palBackGround);

    initMenu();
    initToolBar();
    initConnect();
    mainLayOut();
}


MainWindow::~MainWindow()
{

}

void MainWindow::mainLayOut()
{
    this->main_layout = new QHBoxLayout();
    this->left_main_layout = new QVBoxLayout();
    this->right_main_layout = new QVBoxLayout();

    this->right_camera_layout = new QHBoxLayout();
    this->right_data_layout = new QHBoxLayout();
    this->right_table_layout = new QHBoxLayout();

    this->right_camera_widget = new QWidget(this);
    this->right_data_widget = new QWidget(this);
    this->right_table_widget = new QWidget(this);


    main_layout->addLayout(left_main_layout);
    main_layout->addLayout(right_main_layout);
    main_layout->setStretchFactor(left_main_layout,1);
    main_layout->setStretchFactor(right_main_layout,1);

    //right_main_layout->addWidget(cv4Demo);
    right_main_layout->addLayout(right_camera_layout);
    right_main_layout->addLayout(right_data_layout);
    right_main_layout->addLayout(right_table_layout);
    right_main_layout->setStretchFactor(right_camera_layout,1);
    right_main_layout->setStretchFactor(right_data_layout,1);
    right_main_layout->setStretchFactor(right_table_layout,1);

    right_camera_layout->addWidget(cv4Demo);
    right_data_layout->addWidget(cv4Demo1);
    right_data_layout->addWidget(cv4Demo2);
    right_data_layout->setStretchFactor(cv4Demo1,1);
    right_data_layout->setStretchFactor(cv4Demo2,1);

    right_table_layout->addWidget(echartWindow);
//    right_main_layout->addLayout(right_camera_layout);
////    right_main_layout->addSpacing(10);
////    right_main_layout->addStretch();
//    right_main_layout->addLayout(right_data_layout);
//    right_main_layout->addLayout(right_table_layout);
//    right_main_layout->setStretchFactor(right_camera_layout,1);
//    right_main_layout->setStretchFactor(right_data_layout,1);
//    right_main_layout->setStretchFactor(right_table_layout,1);


    right_main_layout->addWidget(cv4Demo);
    left_main_layout->addWidget(vtkNativeWindow);

    this->main_widget = new QWidget(this);
    main_widget->setLayout(main_layout);
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
    openAct = new QAction(tr("&Open"),this);
    openAct -> setShortcut(tr("ctrl+O"));
    printAct = new QAction(tr("&Print"),this);
    printAct -> setShortcut(tr("ctrl+P"));
    exitAct = new QAction(tr("&Exit"),this);
    exitAct -> setShortcut(tr("ctrl+Q"));
    fileMenu -> addAction(openAct);
    fileMenu -> addAction(printAct);
    fileMenu -> addSeparator();
    fileMenu -> addAction(exitAct);

    zoomInAct = new QAction(tr("Zoom &In"),this);
    zoomInAct -> setShortcut(tr("ctrl+="));
    zoomOutAct = new QAction(tr("Zoom &Out"),this);
    zoomOutAct -> setShortcut(tr("ctrl+-"));
    normalSizeAct = new QAction(tr("&Normal Size"),this);
    normalSizeAct -> setShortcut(tr("ctrl+S"));
    fitToWindowAct = new QAction(tr("&Fit to Window"),this);
    fitToWindowAct -> setShortcut(tr("ctrl+F"));
    viewMenu -> addAction(zoomInAct);
    viewMenu -> addAction(zoomOutAct);
    viewMenu -> addAction(normalSizeAct);
    viewMenu -> addSeparator();
    viewMenu -> addAction(fitToWindowAct);

    aboutAct = new QAction(tr("&About"),this);
    aboutQtAct = new QAction(tr("&About Qt"),this);
    helpMenu -> addAction(aboutAct);
    helpMenu -> addAction(aboutQtAct);

    //add menus to menubar
    menuBar() -> addMenu(fileMenu);
    menuBar() -> addMenu(viewMenu);
    menuBar() -> addMenu(helpMenu);
}

void MainWindow::initToolBar()
{
    //add a toolbar and add its actions
     fileToolBar = new QToolBar(this);

     fileToolBar -> addAction(openAct);
     fileToolBar -> addAction(printAct);
     fileToolBar -> addAction(exitAct);

     addToolBar(Qt::TopToolBarArea,fileToolBar);
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



