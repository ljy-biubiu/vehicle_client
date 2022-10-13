#ifndef DOCUMENT_H
#define DOCUMENT_H

#include <QObject>
#include <QJsonObject>

class Document : public QObject
{
    Q_OBJECT
public:
    explicit Document(QObject *parent = nullptr);

public:
    // 发送json数据给html
    void sendJsonToWeb(const QJsonObject _obj);

signals:
    void receiveTextFromWeb(const QString &test);       // JS给QT发送数据
    void signalToWeb(const QJsonObject &json);          // QT给JS发送数据

public slots:
    void sendTextToQT(const QString &test);

private:
    QJsonObject obj;

signals:

};

#endif // DOCUMENT_H
