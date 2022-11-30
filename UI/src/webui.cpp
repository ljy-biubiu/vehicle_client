#include "webui.h"
#include "ui_webui.h"

WebUi::WebUi(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WebUi)
{
    ui->setupUi(this);
    ui->web_widget->setUrl(QUrl("qrc:/web_ui/dynamic-data.html"));
    //ui->web_widget->setUrl(QUrl("file:///home/ljy/Desktop/test_mymonitoer/PortMonitor1127_2202/PortMonitor2/UI/resource/web_ui/dynamic-data.html"));
}

WebUi::~WebUi()
{
    delete ui;
}
