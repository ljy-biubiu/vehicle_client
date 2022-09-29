#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qvtkopenglwindow.h"

#include <QLabel>
#include <QScrollArea>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>

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

    QVTKOpenGLNativeWindow* vtkNativeWindow;


private:
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
    void open();
    void print();
    void exit();
    void zoomIn();
    void zoomOut();
    void normalSize();
    void fitToWindow();
    void about();
    void aboutQt();



};
#endif // MAINWINDOW_H


