#include "bmapwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    bmapWindow w;
    w.show();
    return a.exec();
}
