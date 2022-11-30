#ifndef ALARM_H
#define ALARM_H

#include <QWidget>
#include <QLabel>
#include <QTimer>

namespace Ui {
class Alarm;
}

class Alarm : public QWidget
{
    Q_OBJECT

public:
    explicit Alarm(QWidget *parent = nullptr);
    ~Alarm();

    void setLED(QLabel* label, int color, int size ,int width);
    QTimer *my_timer;

private:
    Ui::Alarm *ui;
};

#endif // ALARM_H
