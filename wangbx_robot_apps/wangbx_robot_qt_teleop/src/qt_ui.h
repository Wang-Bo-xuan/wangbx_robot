#ifndef __QT_UI_H__
#define __QT_UI_H__

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>

namespace WANGBX_ROBOT
{
  class teleop_UI : public QObject
  {
    Q_OBJECT

    public:
      teleop_UI(int argc,char *argv[]);
      ~teleop_UI();

    private slots:
      void Button_LEFT_UP_CallBack(bool clicked);
      void Button_LEFT_CallBack(bool clicked);
      void Button_LEFT_DOWN_CallBack(bool clicked);
      void Button_UP_CallBack(bool clicked);
      void Button_STOP_CallBack(bool clicked);
      void Button_DOWN_CallBack(bool clicked);
      void Button_RIGHT_UP_CallBack(bool clicked);
      void Button_RIGHT_CallBack(bool clicked);
      void Button_RIGHT_DOWN_CallBack(bool clicked);
      void Button_VEL_INC_CallBack(bool clicked);
      void Button_VEL_DEC_CallBack(bool clicked);
      void Button_LINE_INC_CallBack(bool clicked);
      void Button_LINE_DEC_CallBack(bool clicked);
      void Button_ROT_INC_CallBack(bool clicked);
      void Button_ROT_DEC_CallBack(bool clicked);
      void Publish(void);

    private:
      void Init(int argc, char *argv[]);
      void GetVel(const nav_msgs::Odometry &msg);

      double line_vel_;
      double rot_vel_;
      double cmd_line_vel_;
      double cmd_rot_vel_;

      QApplication *app_;
      QMainWindow *window_;
      QPushButton *btn_up_;
      QPushButton *btn_down_;
      QPushButton *btn_left_;
      QPushButton *btn_right_;
      QPushButton *btn_stop_;
      QPushButton *btn_left_up_;
      QPushButton *btn_left_down_;
      QPushButton *btn_right_up_;
      QPushButton *btn_right_down_;
      QPushButton *btn_vel_inc_;
      QPushButton *btn_vel_dec_;
      QPushButton *btn_rot_inc_;
      QPushButton *btn_rot_dec_;
      QPushButton *btn_line_inc_;
      QPushButton *btn_line_dec_;
      QLabel *label_target_rot_vel_;
      QLabel *label_target_line_vel_;
      QLabel *label_current_rot_vel_;
      QLabel *label_current_line_vel_;
      QTimer *timer_;

      ros::Publisher *cmd_pub_;
      ros::Subscriber *odom_sub_;
      geometry_msgs::Twist vel_;
  };
}

#endif
