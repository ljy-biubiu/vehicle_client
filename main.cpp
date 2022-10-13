#include "mainwindow.h"
#include "cv4demo.h"
#include <QFile>
#include <QDir>
#include <QMutex>

#include <QApplication>
#include <QDesktopWidget>

void createLogFile();
void outputMessage( QtMsgType type, const QMessageLogContext& context, const QString& msg );


/**
 * @brief createLogFile 创建存放log文件的文件夹
 */
void createLogFile() {
    QDir* log = new QDir;

    if ( !log->exists( "./error/" ) ) { log->mkdir( "./error/" ); }
    if ( !log->exists( "./info-log/" ) ) { log->mkdir( "./info-log/" ); }
    if ( !log->exists( "./crittcal-log/" ) ) { log->mkdir( "./crittcal-log/" ); }
    if ( !log->exists( "./debug-log/" ) ) { log->mkdir( "./debug-log/" ); }
}

int main(int argc, char *argv[])
{
    /******************************************** 设置保存日志文件 ********************************************/
    createLogFile();                          // 生成日志文件夹
    qInstallMessageHandler( outputMessage );  // 注册MsgHandler回调函数

    /******************************************** 打开桌面应用 ********************************************/

    QApplication app(argc, argv);
    QDesktopWidget dw;
    MainWindow w;

    /******************************************** 设置qss样式表 ******************************************************/
    QFile file(":/myQss.qss");
    file.open(QIODevice::ReadOnly);
    QString stylesheet = QLatin1String(file.readAll());
    app.setStyleSheet(stylesheet);

//    double x=dw.width()*0.8;
//    double y=dw.height()*0.8;
    w.showMaximized();
    w.show();


    return app.exec();

}


/**
 * @brief outputMessage MsgHandler回调函数,用于向日志文件中输出日志。
 * @param type 信息的类型，{QtDebugMsg，QtInfoMsg，QtWarningMsg，QtCriticalMsg，QtFatalMsg}之一
 * @param context 信息的资料
 * @param msg 信息的文本
 */
void outputMessage( QtMsgType type, const QMessageLogContext& context, const QString& msg ) {

    static QString info_log_file_name     = "./info-log/" + QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) + ".csv";
    static QString crittcal_log_file_name = "./crittcal-log/" + QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) + ".txt";
    static QString debug_log_file_name    = "./debug-log/" + QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) + ".txt";
    static QString log_header( "Time,Msg,"
                               "NAVI,CurRoute,CurId,Dir,AgvState,NaviMode,Mode,"
                               "SPEED,LSet,RSet,Lreal,Rreal,PidAdj,"
                               "COORDINATE,Station,Step,mode,cur x,cur y,cur rad,QRCode id,QRCode x,QRCode y,QRCode rad,slam x,slam y,slam rad,"
                               "CurEncLeft,CurEncRight,EncLeftInc,EncRightInc,DisCur2Target,RadErrStart2Target" );
    Q_UNUSED( context )
    static QMutex mutex;
    mutex.lock();
    QByteArray localMsg = msg.toLocal8Bit();
    QString    msg_type;
    switch ( type ) {
    case QtDebugMsg:
        msg_type = QString( "[Debug]" );
        fprintf( stderr, "Debug: %s\n", localMsg.constData() );
        break;

    case QtInfoMsg:
        msg_type = QString( "[Info]" );
        fprintf( stderr, "Info: %s\n", localMsg.constData() );
        break;

    case QtWarningMsg:
        msg_type = QString( "[Warning]" );
        fprintf( stderr, "Warning: %s \n", localMsg.constData() );
        break;

    case QtCriticalMsg:
        msg_type = QString( "[Critical]" );
        fprintf( stderr, "Critical: %s \n", localMsg.constData() );
        break;

    case QtFatalMsg:
        msg_type = QString( "[Fatal]" );
        fprintf( stderr, "Fatal: %s \n", localMsg.constData() );
        abort();
    }
    /***************************info***************************/
    if ( type == QtInfoMsg ) {
        QFile info_file;
        info_file.setFileName( info_log_file_name );

        if ( info_file.size() > 10 * 1024 * 1024 )  //超过10M,重新再创建一个文件
        {
            info_log_file_name = "./info-log/" + QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) + ".csv";
            info_file.setFileName( info_log_file_name );
        }

        info_file.open( QIODevice::WriteOnly | QIODevice::Append );
        QTextStream info_text_stream( &info_file );
        if ( info_file.size() == 0 )  //新文件加首行
        {
            info_text_stream << log_header << "\r\n";
        }
        info_text_stream << QDateTime::currentDateTime().toString( "MM/dd hh:mm:ss:zzz" ) << "," << msg << "\r\n";
        info_file.flush();
        info_file.close();
    }

    /***************************debug***************************/
    if ( type == QtDebugMsg ) {
        QFile debug_file;
        debug_file.setFileName( debug_log_file_name );

        if ( debug_file.size() > 1 * 1024 * 1024 )  //超过1M,重新再创建一个文件
        {
            debug_log_file_name = "./debug-log/" + QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) + ".txt";
            debug_file.setFileName( debug_log_file_name );
        }
        debug_file.open( QIODevice::WriteOnly | QIODevice::Append );
        QTextStream debug_text_stream( &debug_file );
        debug_text_stream << QDateTime::currentDateTime().toString( "MM/dd hh:mm:ss " ) + msg << "\r\n";
        debug_file.flush();
        debug_file.close();
    }

    /***************************crittcal***************************/
    if ( type == QtCriticalMsg ) {
        QFile crittcal_file;
        crittcal_file.setFileName( crittcal_log_file_name );

        if ( crittcal_file.size() > 1 * 1024 * 1024 )  //超过1M,重新再创建一个文件
        {
            crittcal_log_file_name = "./crittcal-log/" + QDateTime::currentDateTime().toString( "yyyy-MM-dd hh_mm" ) + ".txt";
            crittcal_file.setFileName( crittcal_log_file_name );
        }
        crittcal_file.open( QIODevice::WriteOnly | QIODevice::Append );
        QTextStream crittcal_text_stream( &crittcal_file );
        crittcal_text_stream << QDateTime::currentDateTime().toString( "MM/dd hh:mm:ss " ) + msg << "\r\n";
        crittcal_file.flush();
        crittcal_file.close();
    }

    mutex.unlock();
}
