#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qvtkopenglwindow.h"
#include "cv4demo.h"
#include "bmapwindow.h"

#include <QLabel>
#include <QScrollArea>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QHBoxLayout>
#include <QWidget>

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void initMenu();
    void initToolBar();
    void initConnect();
    void createActions();
    void mainLayOut();

    QVTKOpenGLNativeWindow* vtkNativeWindow;


private:
    //layout
    QHBoxLayout *main_layout;
    QVBoxLayout *left_main_layout;
    QVBoxLayout *right_main_layout;

    QHBoxLayout *right_camera_layout;
    QHBoxLayout *right_data_layout;
    QHBoxLayout *right_table_layout;

    //widget
    QWidget *main_widget;
    QWidget *right_camera_widget;
    QWidget *right_data_widget;
    QWidget *right_table_widget;


    EchartWindow *echartWindow;
    Cv4Demo *cv4Demo;
    Cv4Demo *cv4Demo1;
    Cv4Demo *cv4Demo2;
    QHBoxLayout *layout;
    QWidget *widget;


    QLabel *imageLabel;
    QScrollArea *scrollAera;

    QMenu *fileMenu;
    QMenu *viewMenu;
    QMenu *helpMenu;

    QToolBar *fileToolBar;

    QAction *openAct;
    QAction *printAct;
    QAction *exitAct;
    QAction *zoomInAct;
    QAction *zoomOutAct;
    QAction *normalSizeAct;
    QAction *fitToWindowAct;
    QAction *aboutAct;
    QAction *aboutQtAct;

private slots:




};
#endif // MAINWINDOW_H


