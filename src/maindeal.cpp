#include "maindeal.h"


Maindeal::Maindeal(int my_argc_,char **my_argv_) : my_argc( my_argc_ ),my_argv( my_argv_ )
{
    /******************************************** 设置保存日志文件 *******************************************/
    createLogFile();
    /******************************************** 初始化雷达驱动线程 ******************************************/
    initLidarDrive();
    /******************************************** 初始化摄像头驱动线程 ****************************************/
    initCameraDrive();
    /******************************************** 初始化算法线程 *********************************************/
    //initAlcorithm();
    /******************************************** 初始化下位机通讯线程 ****************************************/
    chatLowerMachine();
    /******************************************** 初始化UI **************************************************/
    initUI();
    /******************************************** 初始化参数服务  ********************************************/
    initParamsServer();
    /********************************************  执行程序   ***********************************************/
    startProcess();
}

void Maindeal::initParamsServer()
{
    params_event = std::make_shared<ParamsEvent<ObserverFunc>>();
    //params_event->initParams();

    params_event->Connect(std::bind(&MainWindow::globalParamsCallback, w, std::placeholders::_1, std::placeholders::_2));
    params_event->Connect(std::bind(&LidarDriveInterface::globalParamsCallback, lidarDriveInterface, std::placeholders::_1, std::placeholders::_2));
    params_event->Connect(std::bind(&PTZ::globalParamsCallback, ptz, std::placeholders::_1, std::placeholders::_2));
    params_event->Connect(std::bind(&ComunicateInterface::globalParamsCallback, comunicateInterface, std::placeholders::_1, std::placeholders::_2));

    comunicateInterface->initParamsEvent(params_event);
    w->initParamsEvent(params_event);
}

void Maindeal::initLog()
{
    createLogFile();
}


//#include "protocol.hpp"

void Maindeal::chatLowerMachine()
{
    comunicateInterface = new ChatPlc();

    //    //test
    //    WriteData a;
    //    a = {0};
    //    a.status.comnunication_flag = 10086.1001;
    //    comunicateInterface->send_data(a);
    //    a.status.comnunication_flag = 100312312386.1001;
    //    comunicateInterface->send_data(a);

}

void Maindeal::initLidarDrive()
{
    lidarDriveInterface = new LidarDrive128S1();
}

void Maindeal::initCameraDrive()
{
    this->ptz = new PTZ();
}


void Maindeal::initUI()
{
    /******************************************** 打开桌面应用 ********************************************/

    app = new QApplication(this->my_argc, this->my_argv);
    dw = new QDesktopWidget();
    w = new MainWindow(ptz,lidarDriveInterface);
    /******************************************** 设置qss样式表 ********************************************/
    QFile file(":/qss/darkblue.css");
    if (file.open(QFile::ReadOnly))
    {
        QString qss = QLatin1String(file.readAll());
        QString paletteColor = qss.mid(20, 7);
        qApp->setPalette(QPalette(QColor(paletteColor)));
        qApp->setStyleSheet(qss);
        file.close();
    }

    w->showMaximized();
    w->show();
}

void Maindeal::startProcess()
{
    this->ptz->PTZ_init((HWND)this->w->getCameraWidget()->winId());
    this->lidarDriveInterface->init();
    this->comunicateInterface->init();
}


const QApplication* Maindeal::getApp()
{
    return app;
}



