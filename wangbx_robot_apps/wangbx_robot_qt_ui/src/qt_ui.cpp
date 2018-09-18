#include "qt_ui.h"

WANGBX_ROBOT::ui::ui(int argc, char *argv[])
{
  this->Init(argc,argv);

  QObject::connect(this->timer_,SIGNAL(timeout()),this,SLOT(Publish()));
  this->timer_->start(50);
  this->label_disp_->installEventFilter(this);

  this->app_->exec();
}

WANGBX_ROBOT::ui::~ui()
{
  if(this->label_disp_)
  {
    delete this->label_disp_;
    this->label_disp_ = NULL;
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
}

void WANGBX_ROBOT::ui::Disp(void)
{
  QPainter painter(this->label_disp_);
  painter.setPen(Qt::gray);
  painter.setBrush(Qt::green);
  painter.drawRect(10,10,200,200);
}

bool WANGBX_ROBOT::ui::eventFilter(QObject *watched, QEvent *event)
{
  if(watched == this->label_disp_ && event->type() == QEvent::Paint)
  {
    this->Disp();
  }

  return QWidget::eventFilter(watched,event);
}

void WANGBX_ROBOT::ui::Init(int argc, char *argv[])
{
  this->app_ = new QApplication(argc,argv);
  this->window_ = new QMainWindow();
  this->window_->resize(1024,768);
  this->window_->setFixedSize(this->window_->width(), this->window_->height());

  this->label_disp_ = new QLabel();
  this->label_disp_->setText("label");
  this->label_disp_->setGeometry(0,0,100,100);
  this->label_disp_->setParent(this->window_);

  this->timer_ = new QTimer(this->window_);

  this->window_->show();
}

void WANGBX_ROBOT::ui::GetLidar(const sensor_msgs::LaserScan &msg)
{
  this->lidar_.angle_increment = msg.angle_increment;
  this->lidar_.angle_min = msg.angle_min;
  this->lidar_.angle_max = msg.angle_max;
  this->lidar_.range_min = msg.range_min;
  this->lidar_.range_max = msg.range_max;
  this->lidar_.ranges.assign(msg.ranges.begin(),msg.ranges.end());
  this->lidar_.intensities.assign(msg.intensities.begin(),msg.intensities.end());



  this->lidar_.ranges.clear();
  this->lidar_.intensities.clear();
}

void WANGBX_ROBOT::ui::Publish(void)
{
  ros::spinOnce();
}
