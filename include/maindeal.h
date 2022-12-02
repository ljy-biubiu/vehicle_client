#ifndef MAINDEAL_H
#define MAINDEAL_H

#include <QApplication>
#include <QDesktopWidget>

#include "mainwindow.h"
#include "ptz.h"
#include "lidardrive128s1.h"
#include "chat_plc.h"
#include "paramsevent.hpp"
#include "my_log.h"
#include "algorithm.h"

class Maindeal
{
public:
    Maindeal(int my_argc_,char **my_argv_);

    //UI
    const QApplication* getApp();
    void initUI();

private:

    void initAlcorithm();
    void chatLowerMachine();
    void initCameraDrive();
    void initLidarDrive();
    void initParamsServer();
    void startProcess();
    void startCamera();
    void initLog();


    int my_argc{0};
    char **my_argv;

    QApplication* app;
    QDesktopWidget* dw;
    MainWindow* w;
    AlgorithmInterface* algorithmInterface;
    PTZ *ptz = nullptr;
    LidarDriveInterface *lidarDriveInterface = nullptr;
    ComunicateInterface *comunicateInterface;
    std::shared_ptr<ParamsEvent<ObserverFunc>> params_event;
};

#endif // MAINDEAL_H
