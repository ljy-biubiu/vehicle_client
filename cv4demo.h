#ifndef CV4DEMO_H
#define CV4DEMO_H

#include <QWidget>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
class cv4Demo : public QWidget
{
public:
    explicit cv4Demo(QWidget *parent = 0);
};

#endif // CV4DEMO_H
