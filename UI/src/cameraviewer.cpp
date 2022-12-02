#include "cameraviewer.h"
#include "ui_cameraviewer.h"

CameraViewer::CameraViewer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraViewer)
{
    ui->setupUi(this);

//    my_widget = new QWidget(this);
//    this->ui->my_layout->addWidget(my_widget);
//    //设置背景黑色
//    QPalette palBackGround(this->palette());
//    palBackGround.setColor(QPalette::Background, QColor(74, 74, 74));
//    this->setAutoFillBackground(true);
//    this->setPalette(palBackGround);
//    this->setStyleSheet("background-color:rgb(74, 74, 74)");



}

CameraViewer::~CameraViewer()
{
    delete ui;
}
