#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    vtkNativeWindow = new QVTKOpenGLNativeWindow();
    setCentralWidget(vtkNativeWindow);
    setWindowTitle(tr("cloud viewer demo"));

    initMenu();
    initToolBar();
    initConnect();
}


MainWindow::~MainWindow()
{

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
    connect(openAct,SIGNAL(triggered()),this,SLOT(open()));
    connect(printAct,SIGNAL(triggered()),this,SLOT(print()));
    connect(exitAct,SIGNAL(triggered()),this,SLOT(exit()));
    connect(zoomInAct,SIGNAL(triggered()),this,SLOT(zoomIn()));
    connect(zoomOutAct,SIGNAL(triggered()),this,SLOT(zoomOut()));
    connect(normalSizeAct,SIGNAL(triggered()),this,SLOT(normalSize()));
    connect(fitToWindowAct,SIGNAL(triggered()),this,SLOT(fitToWindow()));
    connect(aboutAct,SIGNAL(triggered()),this,SLOT(about()));
    connect(aboutQtAct,SIGNAL(triggered()),this,SLOT(aboutQt()));
}

//implement slot functions
void MainWindow::open()
{

}

void MainWindow::print()
{

}

void MainWindow::exit()
{

}

void MainWindow::zoomIn()
{

}

void MainWindow::zoomOut()
{

}

void MainWindow::normalSize()
{

}

void MainWindow::fitToWindow()
{

}

void MainWindow::about()
{

}

void MainWindow::aboutQt()
{

}
