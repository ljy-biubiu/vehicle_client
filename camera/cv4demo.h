#ifndef CV4DEMO_H
#define CV4DEMO_H

#include <QWidget>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QLabel>
#include <QHBoxLayout>


using namespace cv;
class Cv4Demo : public QWidget
{
public:
    explicit Cv4Demo(QWidget *parent = 0);
    QLabel* my_label;
};

#endif // CV4DEMO_H
