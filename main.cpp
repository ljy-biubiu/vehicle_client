#include "mainwindow.h"
#include "cv4demo.h"
#include <QFile>

#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QDesktopWidget dw;
    MainWindow w;

    /******************************************** 设置qss样式表 ******************************************************/
    QFile file(":/myQss.qss");
    file.open(QIODevice::ReadOnly);
    QString stylesheet = QLatin1String(file.readAll());
    app.setStyleSheet(stylesheet);

    int x=dw.width()*0.8;
    int y=dw.height()*0.8;
    w.showMaximized();
    w.show();


    return app.exec();

}
