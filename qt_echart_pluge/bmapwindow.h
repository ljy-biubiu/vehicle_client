#ifndef ECHARTWINDOW_H
#define ECHARTWINDOW_H

#include <QMainWindow>
#include <QWebEngineView>
#include <QWebChannel>

#include "document.h"

QT_BEGIN_NAMESPACE
namespace Ui { class bmapWindow; }
QT_END_NAMESPACE

class EchartWindow : public QMainWindow
{
    Q_OBJECT

public:
    EchartWindow(QWidget *parent = nullptr);
    ~EchartWindow();

private slots:
    void test2_clicked();
    void test3_clicked();
    void onBtnSend_clicked();
    void onReceiveData(const QString &jsonString);
    void onResizeEcharts();

private:
    Ui::bmapWindow *ui;
    bool isLoaded;
    QWebEngineView* view;
    QWebChannel* channel;
    Document document;      // QT和Html通信的中间商

    QJsonObject setUniqueJson(const QString &key, const QString &value);

};
#endif // MAINWINDOW_H
