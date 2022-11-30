﻿#ifndef ADDLIDAR_H
#define ADDLIDAR_H

#include <QDialog>
#include "common.h"

namespace Ui {
class AddLidar;
}

class AddLidar : public QDialog
{
    Q_OBJECT

public:
    explicit AddLidar(QWidget *parent = 0);
    ~AddLidar();
    void initcombox(QList<int>&items);
    void ShowData();
    SetData data;
private slots:
    void on_btn_sure_clicked();

    void on_btn_no_clicked();

signals:
    void SendSet(SetData data);
private:
    Ui::AddLidar *ui;
};

#endif // ADDLIDAR_H
