#include "bmapwindow.h"
#include "ui_bmapwindow.h"
#include "qurl.h"

#include <QtWebEngineWidgets/QWebEngineView>
#include <QResizeEvent>
#include "qfileinfo.h"
#include "qjsonobject.h"
#include "qjsonarray.h"
#include "qjsondocument.h"

EchartWindow::EchartWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::bmapWindow)
{
    ui->setupUi(this);
    channel = new QWebChannel(this);
    // 向web客户端注册对象，使得web客户端可以通过该对象调用QT的槽函数
    // 参数一：web端使用的标识(可以乱写，但必须与web中对应)    参数二：传送过去的QT对象
    channel->registerObject(QString::fromLocal8Bit("doc"), &document);

    // 设置QWebChannel，打通与JavaScript的联系
    ui->widget->page()->setWebChannel(channel);

    // 设置显示的网页链接
    //ui->widget->setUrl(QUrl("qrc:/map-polygon.html"));
    //ui->widget->setUrl(QUrl("qrc:/input_messages_to_qt.html"));
    ui->widget->setUrl(QUrl("qrc:/dynamic-data2.html"));

    //QObject::connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onBtnSend_clicked()));
    // web调用document类的槽函数，在函数内部发射信号，主函数on_receiveData进行相应，处理接收到的数据
    QObject::connect(&document, &Document::receiveTextFromWeb, this, &EchartWindow::onReceiveData);
    QObject::connect(this->ui->widget, SIGNAL(loadFinished(bool)), this, SLOT(onResizeEcharts()));


}


// 自定义一个jsonObject返回
QJsonObject EchartWindow::setUniqueJson(const QString &key, const QString &value)
{
    // 定义json
    QJsonObject json;
    // 将数据绑定到json中
    json.insert(key, value);

    return json;
}

void EchartWindow::onBtnSend_clicked() {
//    // 获取用户输入的内容
//    QString str = ui->lineEdit->text();
//    ui->lineEdit->setText("");

//    if (str == "") {
//        return;
//    }

//    // 设置成json格式
//    QJsonObject json = setUniqueJson("key", str);

//    // 调用document的函数将数据传到那边，在那边进行发送给Html
//    document.sendJsonToWeb(json);
}

// 处理Html发送过来的数据
void EchartWindow::onReceiveData(const QString &jsonString)
{
//    /* 这个字符串是Json类型的，不能直接使用，需要做转化 */

//    // 将字符串转换为QJsonDocument
//    QJsonDocument jsonDocument = QJsonDocument::fromJson(jsonString.toLocal8Bit().data());
//    // 判断是否转换成功，为NULL则转换失败
//    if(jsonDocument.isNull())
//    {
//        qDebug()<< "String NULL ： "<< jsonString.toLocal8Bit().data();
//    }
//    // 获取根 { }
//    QJsonObject jsonObject = jsonDocument.object();

//    // 根据相同的键值获取数据，并显示到QTextEdit中
//    QJsonValue value = jsonObject.value("key");
//    //ui->textEdit->append(value.toString());
//    qDebug()<<"receive datas :"<<value.toString();
//    ui->lineEdit->setText(value.toString());
}


//动态控制地图大小
void EchartWindow::onResizeEcharts()
{
    QJsonObject sizeData;
    sizeData.insert("width", this->ui->widget->width());
    sizeData.insert("height", this->ui->widget->height());
    QString sizeStr = QJsonDocument(sizeData).toJson();
    QString js = QString("setSize(%1)").arg(sizeStr);
    this->ui->widget->page()->runJavaScript(js);
}




EchartWindow::~EchartWindow()
{
    delete ui;
}

void EchartWindow::test2_clicked()
{
    QJsonObject seriesData;
    QJsonArray data1 = {1, 3, 9, 27, 81, 247, 741, 2223, 6669};
    seriesData.insert("data1", data1);
    QJsonArray data2 = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    seriesData.insert("data2", data2);

    QString optionStr = QJsonDocument(seriesData).toJson();
    QString js = QString("init2(%1)").arg(optionStr);
    ui->widget->page()->runJavaScript(js);
}

void EchartWindow::test3_clicked()
{
    isLoaded = true;
    QJsonObject sizeData;
    sizeData.insert("width", ui->widget->width() - 20);
    sizeData.insert("height", ui->widget->height() - 20);
    QString sizeStr = QJsonDocument(sizeData).toJson();
    QString js = QString("setSize(%1)").arg(sizeStr);
    ui->widget->page()->runJavaScript(js);
}
