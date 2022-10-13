#include "document.h"

Document::Document(QObject *parent) : QObject(parent)
{

}


// 给JS发送Json数据
void Document::sendJsonToWeb(const QJsonObject _obj)
{
    // 临时存储当前主程序发送给JS的数据，这里我还定义了一个函数用于返回obj的，只是没有写出来！
    if (this->obj != _obj) {
        obj = _obj;
    }

    // 通过信号带参的的形式进行发送
    emit signalToWeb(_obj);		// 这个是QT发送给JS
}

// JS调用此函数传参，带数据给QT
void Document::sendTextToQT(const QString &test)
{
    // 发射信号，将数据传给主窗体
    emit receiveTextFromWeb(test);		// 这个是JS发送给QT
}
