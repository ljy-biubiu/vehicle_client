#include "addlidar.h"
#include "ui_addlidar.h"
#include "QMetaType"
#include <QIntValidator>
#include <QMessageBox>
#include <QRegExpValidator>
#include <QRegExp>
#include <QDebug>
AddLidar::AddLidar(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddLidar)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/image/leishen.ico"));
    setModal(true);
    setFixedSize(size());
    this->setWindowFlags(Qt::FramelessWindowHint);
    ui->widget->setParentWidget(this);
    ui->widget->setTitleName(QStringLiteral("摄像头设置"));
    ui->comboBox->setEditable(true);
    ui->comboBox->lineEdit()->setAlignment(Qt::AlignCenter);
    ui->comboBox->lineEdit()->setReadOnly(true);
}

AddLidar::~AddLidar()
{
    delete ui;
}

void AddLidar::initcombox(QList<int> &items)
{

}
void AddLidar::ShowData()
{
    ui->JDis->setText(QString("%1").arg(data.setDis));
    ui->JAngle->setText(QString("%1").arg(data.setAng));
    ui->XAngle->setText(QString("%1").arg(data.setXAngle));
    ui->YAngle->setText(QString("%1").arg(data.setYAngle));
    ui->ZAngle->setText(QString("%1").arg(data.setZAngle));
    ui->base_x->setText(QString("%1").arg(data.setBase_X));
    ui->base_y->setText(QString("%1").arg(data.setBase_Y));
    ui->base_z->setText(QString("%1").arg(data.setBase_Z));
    ui->setradius->setText(QString("%1").arg(data.setRadius));
    ui->clusterTolerance->setText(QString("%1").arg(data.clusterTolerance));
    ui->minClusterSize->setText(QString("%1").arg(data.minClusterSize));
    ui->maxClusterSize->setText(QString("%1").arg(data.maxClusterSize));
    ui->resolution->setText(QString("%1").arg(data.resolution));
    ui->difference_threshold->setText(QString("%1").arg(data.difference_threshold));
    ui->comboBox->setCurrentText(data.lidarModel);
    ui->pos_x->setText(QString("%1").arg(data.pos_x));
    ui->pos_y->setText(QString("%1").arg(data.pos_y));
    ui->pos_z->setText(QString("%1").arg(data.pos_z));
    ui->up_x->setText(QString("%1").arg(data.up_x));
    ui->up_y->setText(QString("%1").arg(data.up_y));
    ui->up_z->setText(QString("%1").arg(data.up_z));
    ui->view_x->setText(QString("%1").arg(data.view_x));
    ui->view_y->setText(QString("%1").arg(data.view_y));
    ui->view_z->setText(QString("%1").arg(data.view_z));

    ui->ledIp->setText(QString::fromStdString(data.ledIp));
    ui->lidarPort->setText(QString("%1").arg(data.lidarPort));
    ui->spinBox_map->setValue(data.cloudMapFrame);
    ui->kValue->setText(QString("%1").arg(data.kValue));
    ui->distance->setText(QString("%1").arg(data.distance));
    ui->deviceID->setText(QString::fromStdString(data.deviceId));
    ui->dataAddress->setText(QString::fromStdString(data.dataAddress));
    ui->lidarAddress->setText(QString::fromStdString(data.lidarAddress));
    ui->serverIP->setText(QString::fromStdString(data.serverAddress));
    ui->serverPort->setText(QString("%1").arg(data.serverPort));
}

void AddLidar::on_btn_sure_clicked()
{
    SetData data;
    data.setDis = ui->JDis->text().toFloat();
    data.setAng = ui->JAngle->text().toFloat();//设置预置点时的每份角度
    data.setXAngle = ui->XAngle->text().toFloat();
    data.setYAngle = ui->YAngle->text().toFloat();
    data.setZAngle = ui->ZAngle->text().toFloat();
    data.setBase_X = ui->base_x->text().toFloat();
    data.setBase_Y = ui->base_y->text().toFloat();
    data.setBase_Z = ui->base_z->text().toFloat();
    data.setRadius = ui->setradius->text().toFloat();
    data.clusterTolerance = ui->clusterTolerance->text().toFloat();
    data.minClusterSize = ui->minClusterSize->text().toInt();
    data.maxClusterSize = ui->maxClusterSize->text().toInt();
    data.resolution = ui->resolution->text().toFloat();
    data.difference_threshold = ui->difference_threshold->text().toFloat();
    data.lidarModel = ui->comboBox->currentText();
    data.pos_x = ui->pos_x->text().toFloat();
    data.pos_y = ui->pos_y->text().toFloat();
    data.pos_z = ui->pos_z->text().toFloat();
    data.up_x = ui->up_x->text().toFloat();
    data.up_y = ui->up_y->text().toFloat();
    data.up_z = ui->up_z->text().toFloat();
    data.view_x = ui->view_x->text().toFloat();
    data.view_y = ui->view_y->text().toFloat();
    data.view_z = ui->view_z->text().toFloat();
    data.ledIp = ui->ledIp->text().toStdString();
    data.lidarPort = ui->lidarPort->text().toInt();
    data.cloudMapFrame = ui->spinBox_map->value();
    data.kValue = ui->kValue->text().toFloat();
    data.distance = ui->distance->text().toFloat();
    data.deviceId = ui->deviceID->text().toStdString();
    data.dataAddress = ui->dataAddress->text().toStdString();
    data.lidarAddress = ui->lidarAddress->text().toStdString();
    std::cout<<data.lidarAddress<<std::endl;
    std::cout<<data.lidarAddress<<std::endl;
    std::cout<<data.lidarAddress<<std::endl;
    std::cout<<data.lidarAddress<<std::endl;
    data.serverAddress = ui->serverIP->text().toStdString();
    data.serverPort = ui->serverPort->text().toInt();
    emit SendSet(data);
    this->close();
}

void AddLidar::on_btn_no_clicked()
{
    this->close();
}
