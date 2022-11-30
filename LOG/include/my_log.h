#ifndef MYLOG_H
#define MYLOG_H

#include <QFile>
#include <QDir>
#include <QMutex>
#include <QDateTime>
#include <QTextStream>


void createLogFile();
static void outputMessage( QtMsgType type, const QMessageLogContext& context, const QString& msg );


#endif // MYLOG_H
