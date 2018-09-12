#include "qt_ui.h"

WANGBX_ROBOT::teleop_UI::teleop_UI(int argc, char *argv[])
{
  this->Init(argc,argv);

  QObject::connect(this->btn_left_up_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_LEFT_UP_CallBack(bool)));
  QObject::connect(this->btn_left_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_LEFT_CallBack(bool)));
  QObject::connect(this->btn_left_down_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_LEFT_DOWN_CallBack(bool)));
  QObject::connect(this->btn_up_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_UP_CallBack(bool)));
  QObject::connect(this->btn_stop_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_STOP_CallBack(bool)));
  QObject::connect(this->btn_down_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_DOWN_CallBack(bool)));
  QObject::connect(this->btn_right_up_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_RIGHT_UP_CallBack(bool)));
  QObject::connect(this->btn_right_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_RIGHT_CallBack(bool)));
  QObject::connect(this->btn_right_down_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_RIGHT_DOWN_CallBack(bool)));
  QObject::connect(this->btn_vel_inc_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_VEL_INC_CallBack(bool)));
  QObject::connect(this->btn_vel_dec_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_VEL_DEC_CallBack(bool)));
  QObject::connect(this->btn_line_inc_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_LINE_INC_CallBack(bool)));
  QObject::connect(this->btn_line_dec_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_LINE_DEC_CallBack(bool)));
  QObject::connect(this->btn_rot_inc_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_ROT_INC_CallBack(bool)));
  QObject::connect(this->btn_rot_dec_,SIGNAL(clicked(bool)),
                   this,SLOT(Button_ROT_DEC_CallBack(bool)));

  QObject::connect(this->timer_,SIGNAL(timeout()),this,SLOT(Publish()));
  this->timer_->start(50);

  this->app_->exec();
}

WANGBX_ROBOT::teleop_UI::~teleop_UI()
{
  if(this->btn_left_)
  {
    delete this->btn_left_;
    this->btn_left_ = NULL;
  }

  if(this->btn_right_)
  {
    delete this->btn_right_;
    this->btn_right_ = NULL;
  }

  if(this->btn_up_)
  {
    delete this->btn_up_;
    this->btn_up_ = NULL;
  }

  if(this->btn_down_)
  {
    delete this->btn_down_;
    this->btn_down_ = NULL;
  }

  if(this->btn_stop_)
  {
    delete this->btn_stop_;
    this->btn_stop_ = NULL;
  }

  if(this->btn_left_up_)
  {
    delete this->btn_left_up_;
    this->btn_left_up_ = NULL;
  }

  if(this->btn_left_down_)
  {
    delete this->btn_left_down_;
    this->btn_left_down_ = NULL;
  }

  if(this->btn_right_up_)
  {
    delete this->btn_right_up_;
    this->btn_right_up_ = NULL;
  }

  if(this->btn_right_down_)
  {
    delete this->btn_right_down_;
    this->btn_right_down_ = NULL;
  }

  if(this->btn_vel_inc_)
  {
    delete this->btn_vel_inc_;
    this->btn_vel_inc_ = NULL;
  }

  if(this->btn_vel_dec_)
  {
    delete this->btn_vel_dec_;
    this->btn_vel_dec_ = NULL;
  }

  if(this->btn_line_inc_)
  {
    delete this->btn_line_inc_;
    this->btn_line_inc_ = NULL;
  }

  if(this->btn_line_dec_)
  {
    delete this->btn_line_dec_;
    this->btn_line_dec_ = NULL;
  }

  if(this->btn_rot_inc_)
  {
    delete this->btn_rot_inc_;
    this->btn_rot_inc_ = NULL;
  }

  if(this->btn_rot_dec_)
  {
    delete this->btn_rot_dec_;
    this->btn_rot_dec_ = NULL;
  }

  if(this->label_current_line_vel_)
  {
    delete this->label_current_line_vel_;
    this->label_current_line_vel_ = NULL;
  }

  if(this->label_current_rot_vel_)
  {
    delete this->label_current_rot_vel_;
    this->label_current_rot_vel_ = NULL;
  }

  if(this->label_target_line_vel_)
  {
    delete this->label_target_line_vel_;
    this->label_target_line_vel_ = NULL;
  }

  if(this->label_target_rot_vel_)
  {
    delete this->label_target_rot_vel_;
    this->label_target_rot_vel_ = NULL;
  }

  if(this->window_)
  {
    delete this->window_;
    this->window_ = NULL;
  }

  if(this->app_)
  {
    delete this->app_;
    this->app_ = NULL;
  }

  if(this->odom_sub_)
  {
    delete this->odom_sub_;
    this->odom_sub_ = NULL;
  }

  if(this->cmd_pub_)
  {
    delete this->cmd_pub_;
    this->cmd_pub_ = NULL;
  }
}

void WANGBX_ROBOT::teleop_UI::Init(int argc, char *argv[])
{
  this->cmd_line_vel_ = 0.0;
  this->cmd_rot_vel_ = 0.0;
  this->line_vel_ = 1.0;
  this->rot_vel_ = 3.14;

  this->app_ = new QApplication(argc,argv);
  this->window_ = new QMainWindow();
  this->window_->resize(1024,768);
  this->window_->setFixedSize(this->window_->width(), this->window_->height());

  QFont label;
  label.setFamily(QString::fromUtf8("Tahoma"));
  label.setPointSize(12);
  label.setBold(true);
  label.setWeight(75);

  QFont button;
  button.setFamily(QString::fromUtf8("Tahoma"));
  button.setPointSize(12);
  button.setBold(true);
  button.setWeight(75);

  this->btn_left_up_ = new QPushButton();
  this->btn_left_up_->setText("left_up");
  this->btn_left_up_->setGeometry(592,284,100,100);
  this->btn_left_up_->setParent(this->window_);
  this->btn_left_up_->setAutoRepeat (true);
  this->btn_left_up_->setAutoRepeatDelay(50);
  this->btn_left_up_->setAutoRepeatInterval(50);
  this->btn_left_up_->setFont(button);

  this->btn_left_ = new QPushButton();
  this->btn_left_->setText("left");
  this->btn_left_->setGeometry(592,434,100,100);
  this->btn_left_->setParent(this->window_);
  this->btn_left_->setAutoRepeat (true);
  this->btn_left_->setAutoRepeatDelay(50);
  this->btn_left_->setAutoRepeatInterval(50);
  this->btn_left_->setFont(button);

  this->btn_left_down_ = new QPushButton();
  this->btn_left_down_->setText("left_down");
  this->btn_left_down_->setGeometry(592,584,100,100);
  this->btn_left_down_->setParent(this->window_);
  this->btn_left_down_->setAutoRepeat (true);
  this->btn_left_down_->setAutoRepeatDelay(50);
  this->btn_left_down_->setAutoRepeatInterval(50);
  this->btn_left_down_->setFont(button);

  this->btn_up_ = new QPushButton();
  this->btn_up_->setText("up");
  this->btn_up_->setGeometry(742,284,100,100);
  this->btn_up_->setParent(this->window_);
  this->btn_up_->setAutoRepeat (true);
  this->btn_up_->setAutoRepeatDelay(50);
  this->btn_up_->setAutoRepeatInterval(50);
  this->btn_up_->setFont(button);

  this->btn_stop_ = new QPushButton();
  this->btn_stop_->setText("stop");
  this->btn_stop_->setGeometry(742,434,100,100);
  this->btn_stop_->setParent(this->window_);
  this->btn_stop_->setAutoRepeat (true);
  this->btn_stop_->setAutoRepeatDelay(50);
  this->btn_stop_->setAutoRepeatInterval(50);
  this->btn_stop_->setFont(button);

  this->btn_down_ = new QPushButton();
  this->btn_down_->setText("down");
  this->btn_down_->setGeometry(742,584,100,100);
  this->btn_down_->setParent(this->window_);
  this->btn_down_->setAutoRepeat (true);
  this->btn_down_->setAutoRepeatDelay(50);
  this->btn_down_->setAutoRepeatInterval(50);
  this->btn_down_->setFont(button);

  this->btn_right_up_ = new QPushButton();
  this->btn_right_up_->setText("right_up");
  this->btn_right_up_->setGeometry(892,284,100,100);
  this->btn_right_up_->setParent(this->window_);
  this->btn_right_up_->setAutoRepeat (true);
  this->btn_right_up_->setAutoRepeatDelay(50);
  this->btn_right_up_->setAutoRepeatInterval(50);
  this->btn_right_up_->setFont(button);

  this->btn_right_ = new QPushButton();
  this->btn_right_->setText("right");
  this->btn_right_->setGeometry(892,434,100,100);
  this->btn_right_->setParent(this->window_);
  this->btn_right_->setAutoRepeat (true);
  this->btn_right_->setAutoRepeatDelay(50);
  this->btn_right_->setAutoRepeatInterval(50);
  this->btn_right_->setFont(button);

  this->btn_right_down_ = new QPushButton();
  this->btn_right_down_->setText("right_down");
  this->btn_right_down_->setGeometry(892,584,100,100);
  this->btn_right_down_->setParent(this->window_);
  this->btn_right_down_->setAutoRepeat (true);
  this->btn_right_down_->setAutoRepeatDelay(50);
  this->btn_right_down_->setAutoRepeatInterval(50);
  this->btn_right_down_->setFont(button);

  this->btn_vel_inc_ = new QPushButton();
  this->btn_vel_inc_->setText("vel_inc");
  this->btn_vel_inc_->setGeometry(52,359,100,100);
  this->btn_vel_inc_->setParent(this->window_);
  this->btn_vel_inc_->setAutoRepeat (true);
  this->btn_vel_inc_->setAutoRepeatDelay(50);
  this->btn_vel_inc_->setAutoRepeatInterval(50);
  this->btn_vel_inc_->setFont(button);

  this->btn_vel_dec_ = new QPushButton();
  this->btn_vel_dec_->setText("vel_dec");
  this->btn_vel_dec_->setGeometry(52,509,100,100);
  this->btn_vel_dec_->setParent(this->window_);
  this->btn_vel_dec_->setAutoRepeat (true);
  this->btn_vel_dec_->setAutoRepeatDelay(50);
  this->btn_vel_dec_->setAutoRepeatInterval(50);
  this->btn_vel_dec_->setFont(button);

  this->btn_line_inc_ = new QPushButton();
  this->btn_line_inc_->setText("line_inc");
  this->btn_line_inc_->setGeometry(202,359,100,100);
  this->btn_line_inc_->setParent(this->window_);
  this->btn_line_inc_->setAutoRepeat (true);
  this->btn_line_inc_->setAutoRepeatDelay(50);
  this->btn_line_inc_->setAutoRepeatInterval(50);
  this->btn_line_inc_->setFont(button);

  this->btn_line_dec_ = new QPushButton();
  this->btn_line_dec_->setText("line_dec");
  this->btn_line_dec_->setGeometry(202,509,100,100);
  this->btn_line_dec_->setParent(this->window_);
  this->btn_line_dec_->setAutoRepeat (true);
  this->btn_line_dec_->setAutoRepeatDelay(50);
  this->btn_line_dec_->setAutoRepeatInterval(50);
  this->btn_line_dec_->setFont(button);

  this->btn_rot_inc_ = new QPushButton();
  this->btn_rot_inc_->setText("rot_inc");
  this->btn_rot_inc_->setGeometry(352,359,100,100);
  this->btn_rot_inc_->setParent(this->window_);
  this->btn_rot_inc_->setAutoRepeat (true);
  this->btn_rot_inc_->setAutoRepeatDelay(50);
  this->btn_rot_inc_->setAutoRepeatInterval(50);
  this->btn_rot_inc_->setFont(button);

  this->btn_rot_dec_ = new QPushButton();
  this->btn_rot_dec_->setText("rot_dec");
  this->btn_rot_dec_->setGeometry(352,509,100,100);
  this->btn_rot_dec_->setParent(this->window_);
  this->btn_rot_dec_->setAutoRepeat (true);
  this->btn_rot_dec_->setAutoRepeatDelay(50);
  this->btn_rot_dec_->setAutoRepeatInterval(50);
  this->btn_rot_dec_->setFont(button);

  char tt[100];
  this->label_current_line_vel_ = new QLabel();
  sprintf(tt,"Current Line Vel:%.2lf",this->cmd_line_vel_);
  this->label_current_line_vel_->setText(QString(tt));
  this->label_current_line_vel_->setGeometry(692,35,200,100);
  this->label_current_line_vel_->setParent(this->window_);
  this->label_current_line_vel_->setFont(label);

  this->label_current_rot_vel_ = new QLabel();
  sprintf(tt,"Current Rot Vel:%.2lf",this->cmd_rot_vel_);
  this->label_current_rot_vel_->setText(QString(tt));
  this->label_current_rot_vel_->setGeometry(692,155,200,100);
  this->label_current_rot_vel_->setParent(this->window_);
  this->label_current_rot_vel_->setFont(label);

  this->label_target_line_vel_ = new QLabel();
  sprintf(tt,"Target Line Vel:%.2lf",this->rot_vel_);
  this->label_target_line_vel_->setText(tt);
  this->label_target_line_vel_->setGeometry(152,35,200,100);
  this->label_target_line_vel_->setParent(this->window_);
  this->label_target_line_vel_->setFont(label);

  this->label_target_rot_vel_ = new QLabel();
  sprintf(tt,"Target Rot Vel:%.2lf",this->line_vel_);
  this->label_target_rot_vel_->setText(tt);
  this->label_target_rot_vel_->setGeometry(152,155,200,100);
  this->label_target_rot_vel_->setParent(this->window_);
  this->label_target_rot_vel_->setFont(label);

  this->timer_ = new QTimer(this->window_);

  this->window_->show();

  ros::NodeHandle nh;

  nh.param("line_min",this->line_min_,0.05);
  nh.param("line_max",this->line_max_,2.0);
  nh.param("rot_min",this->rot_min_,1.57);
  nh.param("rot_max",this->rot_max_,6.28);
  nh.param("line_gain",this->line_gain_,0.1);
  nh.param("rot_gain",this->rot_gain_,0.1);

  this->cmd_pub_ = new ros::Publisher();
  *this->cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",20);
  this->odom_sub_ = new ros::Subscriber();
  *this->odom_sub_ = nh.subscribe("/odom",1,&teleop_UI::GetVel,this);
}

void WANGBX_ROBOT::teleop_UI::GetVel(const nav_msgs::Odometry &msg)
{
  this->vel_.linear.x = msg.twist.twist.linear.x;
  this->vel_.angular.z = msg.twist.twist.angular.z;
}

void WANGBX_ROBOT::teleop_UI::Publish(void)
{
  ros::spinOnce();

  geometry_msgs::Twist vel;
  vel.linear.x = this->cmd_line_vel_;
  vel.angular.z = this->cmd_rot_vel_;
  this->cmd_pub_->publish(vel);

  char tt[100];
  sprintf(tt,"target Line Vel:%.2lf",this->line_vel_);
  this->label_target_line_vel_->setText(QString(tt));
  sprintf(tt,"target Rot Vel:%.2lf",this->rot_vel_);
  this->label_target_rot_vel_->setText(QString(tt));
  sprintf(tt,"Current Line Vel:%.2lf",this->vel_.linear.x);
  this->label_current_line_vel_->setText(QString(tt));
  sprintf(tt,"Current Rot Vel:%.2lf",this->vel_.angular.z);
  this->label_current_rot_vel_->setText(QString(tt));

  this->cmd_line_vel_ = 0.0;
  this->cmd_rot_vel_ = 0.0;
}

void WANGBX_ROBOT::teleop_UI::Button_LEFT_UP_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = fabs(this->rot_vel_);
  this->cmd_line_vel_ = fabs(this->line_vel_);
}

void WANGBX_ROBOT::teleop_UI::Button_LEFT_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = fabs(this->rot_vel_);
  this->cmd_line_vel_ = 0.0;
}

void WANGBX_ROBOT::teleop_UI::Button_LEFT_DOWN_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = -fabs(this->rot_vel_);
  this->cmd_line_vel_ = -fabs(this->line_vel_);
}

void WANGBX_ROBOT::teleop_UI::Button_UP_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = 0.0;
  this->cmd_line_vel_ = fabs(this->line_vel_);
}

void WANGBX_ROBOT::teleop_UI::Button_STOP_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = 0.0;
  this->cmd_line_vel_ = 0.0;
}

void WANGBX_ROBOT::teleop_UI::Button_DOWN_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = 0.0;
  this->cmd_line_vel_ = -fabs(this->line_vel_);
}

void WANGBX_ROBOT::teleop_UI::Button_RIGHT_UP_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = -fabs(this->rot_vel_);
  this->cmd_line_vel_ = fabs(this->line_vel_);
}

void WANGBX_ROBOT::teleop_UI::Button_RIGHT_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = -fabs(this->rot_vel_);
  this->cmd_line_vel_ = 0.0;
}

void WANGBX_ROBOT::teleop_UI::Button_RIGHT_DOWN_CallBack(bool clicked)
{
  this->cmd_rot_vel_ = fabs(this->rot_vel_);
  this->cmd_line_vel_ = -fabs(this->line_vel_);
}

void WANGBX_ROBOT::teleop_UI::Button_VEL_INC_CallBack(bool clicked)
{
  this->rot_vel_ *= (1.0 + this->rot_gain_);
  this->line_vel_ *= (1.0 + this->line_gain_);

  if(this->rot_vel_ > this->rot_max_)
  {
    this->rot_vel_ = this->rot_max_;
  }

  if(this->line_vel_ > this->line_max_)
  {
    this->line_vel_ = this->line_max_;
  }
}

void WANGBX_ROBOT::teleop_UI::Button_VEL_DEC_CallBack(bool clicked)
{
  this->rot_vel_ *= (1 - this->rot_gain_);
  this->line_vel_ *= (1 - this->line_gain_);

  if(this->rot_vel_ < this->rot_min_)
  {
    this->rot_vel_ = this->rot_min_;
  }

  if(this->line_vel_ < this->line_min_)
  {
    this->line_vel_ = this->line_min_;
  }
}

void WANGBX_ROBOT::teleop_UI::Button_LINE_INC_CallBack(bool clicked)
{
  this->line_vel_ *= (1.0 + this->line_gain_);

  if(this->line_vel_ > this->line_max_)
  {
    this->line_vel_ = this->line_max_;
  }
}

void WANGBX_ROBOT::teleop_UI::Button_LINE_DEC_CallBack(bool clicked)
{
  this->line_vel_ *= (1 - this->line_gain_);

  if(this->line_vel_ < this->line_min_)
  {
    this->line_vel_ = this->line_min_;
  }
}

void WANGBX_ROBOT::teleop_UI::Button_ROT_INC_CallBack(bool clicked)
{
  this->rot_vel_ *= (1.0 + this->rot_gain_);

  if(this->rot_vel_ > this->rot_max_)
  {
    this->rot_vel_ = this->rot_max_;
  }
}

void WANGBX_ROBOT::teleop_UI::Button_ROT_DEC_CallBack(bool clicked)
{
  this->rot_vel_ *= (1 - this->rot_gain_);

  if(this->rot_vel_ < this->rot_min_)
  {
    this->rot_vel_ = this->rot_min_;
  }
}
