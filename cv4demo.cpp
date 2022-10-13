#include "cv4demo.h"


Cv4Demo::Cv4Demo(QWidget *parent)
    : QWidget(parent)
{
    Mat srcImg = imread("/home/ljy/1.jpg", IMREAD_UNCHANGED);    //加载原始图像

    if (srcImg.empty()) {
        printf("could not load image...");
        return;
    }

    Mat temp, grayImg;
    my_label = new QLabel(this);
//    QHBoxLayout *layout = new QHBoxLayout();
//    layout->addWidget(my_label);

//    setLayout(layout);
    cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
//    //cvtColor(srcImg, temp, CV_BGR2RGB);//BGR convert to RGB
    cvtColor(grayImg, temp, COLOR_GRAY2RGB);//GRAY convert to RGB
    QImage Qtemp = QImage((const unsigned char *)(temp.data), temp.cols, temp.rows, temp.step,
                          QImage::Format_RGB888);
    this->my_label->setPixmap(QPixmap::fromImage(Qtemp));
    this->my_label->resize(Qtemp.size());
    //this->my_label->resize(this->size());

//    ui->label->setPixmap(QPixmap::fromImage(Qtemp));
//    ui->label->resize(Qtemp.size());
//    ui->label->show();

}


