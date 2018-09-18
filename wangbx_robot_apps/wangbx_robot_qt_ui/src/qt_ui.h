#ifndef __QT_UI_H__
#define __QT_UI_H__

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QPainter>
#include <QTimer>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <string>

namespace WANGBX_ROBOT
{
  class ui : public QWidget
  {
    Q_OBJECT

    public:
      ui(int argc,char *argv[]);
      ~ui();

    private slots:
      void Publish(void);
      void Disp(void);

    protected:
      bool eventFilter(QObject *object, QEvent *e);

    private:
      void Init(int argc, char *argv[]);
      void GetLidar(const sensor_msgs::LaserScan &msg);

      QApplication *app_;
      QMainWindow *window_;
      QTimer *timer_;
      QLabel *label_disp_;

      sensor_msgs::LaserScan lidar_;
  };
}

#endif
