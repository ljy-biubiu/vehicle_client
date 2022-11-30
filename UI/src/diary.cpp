#include "diary.h"
#include "ui_diary.h"

Diary::Diary(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Diary)
{
    ui->setupUi(this);

    //设置背景黑色
    QPalette palBackGround(this->palette());
    palBackGround.setColor(QPalette::Background, QColor(34, 34, 34));
    this->setAutoFillBackground(true);
    this->setPalette(palBackGround);

    //this->ui->textEdit->setStyleSheet("background-image:url(:/bmp/DSCN1604.JPG)");
    //this->ui->textEdit->setStyleSheet(QString("font-size:13px;border：1px solid black; background：#8c9092"));
    //setStyleSheet("font-size:13px;border：1px solid black; background：#8c9092");

    //        ui->textEdit->setStyleSheet("body {background-color: #aaa}");
}

Diary::~Diary()
{
    delete ui;
}
