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
    //setFixedSize(size());
    this->setWindowFlags(Qt::FramelessWindowHint);
    //ui->widget->setParentWidget(this);
    //ui->widget->setTitleName(QStringLiteral("摄像头设置"));
    ui->comboBox->setEditable(true);
    ui->comboBox->lineEdit()->setAlignment(Qt::AlignCenter);
    ui->comboBox->lineEdit()->setReadOnly(true);
//    ShowData();


}

AddLidar::~AddLidar()
{
    delete ui;
}

void AddLidar::initcombox(QList<int> &items)
{

}
void AddLidar::ShowData(TotalParams &msg)
{
    msg_ = msg;
    //ui->view_x_3->setText(QString("%1").arg(data.setDis));

    //ui->JDis->setText(QString("%1").arg(data.setDis));
    //ui->JAngle->setText(QString("%1").arg(msg_.lidarPreset.  .setAng));
    ui->XAngle->setText(QString("%1").arg(msg_.lidarPreset.XAngle));
    ui->YAngle->setText(QString("%1").arg(msg_.lidarPreset.YAngle));
    ui->ZAngle->setText(QString("%1").arg(msg_.lidarPreset.ZAngle));

    ui->base_x->setText(QString("%1").arg(msg_.lidarPreset.Base_X));
    ui->base_y->setText(QString("%1").arg(msg_.lidarPreset.Base_Y));
    ui->base_z->setText(QString("%1").arg(msg_.lidarPreset.Base_Z));

    ui->setradius->setText(QString("%1").arg(msg_.lidarPreset.paintarea_Radius));
    ui->clusterTolerance->setText(QString("%1").arg(msg_.lidarCluster.ClusterTolerance));

    ui->minClusterSize->setText(QString("%1").arg(msg_.lidarCluster.MinClusterSize));
    ui->maxClusterSize->setText(QString("%1").arg(msg_.lidarCluster.MaxClusterSize));
    ui->resolution->setText(QString("%1").arg(msg_.lidarCluster.Resolution));
    ui->difference_threshold->setText(QString("%1").arg(msg_.lidarCluster.Difference_threshold));

    ui->comboBox->setCurrentText(QString(msg_.lidarBase.LidarModel));
    ui->pos_x->setText(QString("%1").arg(msg_.lidarCameraBase.Pos_X));
    ui->pos_y->setText(QString("%1").arg(msg_.lidarCameraBase.Pos_Y));
    ui->pos_z->setText(QString("%1").arg(msg_.lidarCameraBase.Pos_Z));
    ui->up_x->setText(QString("%1").arg(msg_.lidarCameraBase.Up_X));
    ui->up_y->setText(QString("%1").arg(msg_.lidarCameraBase.Up_Y));
    ui->up_z->setText(QString("%1").arg(msg_.lidarCameraBase.Up_Z));
    ui->view_x->setText(QString("%1").arg(msg_.lidarCameraBase.View_X));
    ui->view_y->setText(QString("%1").arg(msg_.lidarCameraBase.Up_Y));
    ui->view_z->setText(QString("%1").arg(msg_.lidarCameraBase.View_Z));

    ui->lidarPort->setText(QString("%1").arg(msg_.lidarBase.lidarPort));
    ui->kValue->setText(QString("%1").arg(msg_.lidarCluster.Kvalue));

    ui->dataAddress->setText(QString::fromStdString(msg_.lidarBase.DataAddress));
    ui->lidarAddress->setText(QString::fromStdString(msg_.lidarBase.LidarAddress));
    ui->serverIP->setText(QString::fromStdString(msg_.lidarBase.ServerAddress));
    ui->serverPort->setText(QString("%1").arg(msg_.lidarBase.ServerPort));





    ui->camera_ip->setText(QString::fromStdString(msg_.cameraBase.ip));
    ui->camera_name->setText(QString::fromStdString(msg_.cameraBase.name));
    ui->camera_key->setText(QString::fromStdString(msg_.cameraBase.code));
    ui->lower_machine_ip->setText(QString::fromStdString(msg_.lowerMachineBase.ip));
    ui->lower_machine_port->setText(QString("%1").arg(msg_.lowerMachineBase.port));


}

void AddLidar::on_btn_sure_clicked()
{
//    data.setDis = ui->JDis->text().toFloat();
//    data.setAng = ui->JAngle->text().toFloat();//设置预置点时的每份角度
    msg_.lidarPreset.XAngle = ui->XAngle->text().toFloat();
    msg_.lidarPreset.YAngle = ui->YAngle->text().toFloat();
    msg_.lidarPreset.ZAngle = ui->ZAngle->text().toFloat();
    msg_.lidarPreset.Base_X = ui->base_x->text().toFloat();
    msg_.lidarPreset.Base_Y = ui->base_y->text().toFloat();
    msg_.lidarPreset.Base_Z = ui->base_z->text().toFloat();

    msg_.lidarPreset.paintarea_Radius = ui->setradius->text().toFloat();
    msg_.lidarCluster.ClusterTolerance = ui->clusterTolerance->text().toFloat();
    msg_.lidarCluster.MinClusterSize = ui->minClusterSize->text().toInt();

    msg_.lidarCluster.MaxClusterSize = ui->maxClusterSize->text().toInt();
    msg_.lidarCluster.Resolution = ui->resolution->text().toFloat();
    msg_.lidarCluster.Difference_threshold = ui->difference_threshold->text().toFloat();

    msg_.lidarBase.LidarModel = ui->comboBox->currentText().toInt();
    msg_.lidarCameraBase.Pos_X = ui->pos_x->text().toFloat();
    msg_.lidarCameraBase.Pos_Y = ui->pos_y->text().toFloat();
    msg_.lidarCameraBase.Pos_Z = ui->pos_z->text().toFloat();
    msg_.lidarCameraBase.Up_X = ui->up_x->text().toFloat();
    msg_.lidarCameraBase.Up_Y = ui->up_y->text().toFloat();
    msg_.lidarCameraBase.Up_Z = ui->up_z->text().toFloat();
    msg_.lidarCameraBase.View_X = ui->view_x->text().toFloat();
    msg_.lidarCameraBase.Up_Y = ui->view_y->text().toFloat();
    msg_.lidarCameraBase.View_Z = ui->view_z->text().toFloat();

    msg_.lidarBase.lidarPort = ui->lidarPort->text().toInt();
    msg_.lidarCluster.Kvalue = ui->kValue->text().toFloat();

    //data.distance = ui->distance->text().toFloat();
    msg_.lidarBase.DataAddress = ui->dataAddress->text().toStdString();
    msg_.lidarBase.LidarAddress = ui->lidarAddress->text().toStdString();
    msg_.lidarBase.ServerAddress = ui->serverIP->text().toStdString();
    msg_.lidarBase.ServerPort = ui->serverPort->text().toInt();


    msg_.cameraBase.ip = ui->camera_ip->text().toStdString();
    msg_.cameraBase.name = ui->camera_name->text().toStdString();
    msg_.cameraBase.code = ui->camera_key->text().toStdString();

    msg_.lowerMachineBase.ip = ui->lower_machine_ip->text().toStdString();
    msg_.lowerMachineBase.port = ui->lower_machine_port->text().toInt();

    emit SendSet(msg_);
    this->close();
}

void AddLidar::on_btn_no_clicked()
{
    this->close();
}
