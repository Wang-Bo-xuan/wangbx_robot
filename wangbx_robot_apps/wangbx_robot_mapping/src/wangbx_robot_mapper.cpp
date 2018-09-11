#include <wangbx_robot_mapping/wangbx_robot_mapper.h>

WANGBX_ROBOT::Mapper::Mapper(ros::NodeHandle *nh)
{
  this->lidar_sub_ = new ros::Subscriber();
  *this->lidar_sub_ = nh->subscribe("/scan",1,&Mapper::LidarCallBack,this);
  this->odom_sub_ = new ros::Subscriber();
  *this->odom_sub_ = nh->subscribe("/odom",1,&Mapper::OdomCallBack,this);
  this->map_pub_ = new ros::Publisher();
  *this->map_pub_ = nh->advertise<nav_msgs::OccupancyGrid>("/map",10);

  this->tf_broadcaster_ = new tf::TransformBroadcaster();
  this->tf_listener_ = new tf::TransformListener();

  this->Init();
}

WANGBX_ROBOT::Mapper::~Mapper()
{
  if(this->lidar_sub_)
  {
    delete this->lidar_sub_;
    this->lidar_sub_ = NULL;
  }

  if(this->odom_sub_)
  {
    delete this->odom_sub_;
    this->odom_sub_ = NULL;
  }

  if(this->map_pub_)
  {
    delete this->map_pub_;
    this->map_pub_ = NULL;
  }

  if(this->tf_broadcaster_)
  {
    delete this->tf_broadcaster_;
    this->tf_broadcaster_ = NULL;
  }

  if(this->tf_listener_)
  {
    delete this->tf_listener_;
    this->tf_listener_ = NULL;
  }
}

void WANGBX_ROBOT::Mapper::Init(void)
{
  this->GetParam();
}

void WANGBX_ROBOT::Mapper::GetParam(void)
{
  this->map_height_ = 100;
  this->map_width_ = 100;
  this->map_yaw_ = 0;
  this->map_resolution_ = 0.05;
  this->map_frame_ = "map";
  this->odom_frame_ = "odom";

  this->occ_thrshold_ = 100;
  this->free_threshold_ = 0;
  this->unknown_threshold_ = -1;
}

void WANGBX_ROBOT::Mapper::TF(double x,double y,double yaw,string from_frame,string to_frame)
{
  geometry_msgs::PoseStamped from,to;
  from.header.frame_id = from_frame.data();
  from.header.stamp = ros::Time();
  from.pose.position.x = x;
  from.pose.position.y = y;
  from.pose.position.z = 0;
  from.pose.orientation.x = tf::createQuaternionFromYaw(yaw).getX();
  from.pose.orientation.y = tf::createQuaternionFromYaw(yaw).getY();
  from.pose.orientation.z = tf::createQuaternionFromYaw(yaw).getZ();
  from.pose.orientation.w = tf::createQuaternionFromYaw(yaw).getW();

  try
  {
    this->tf_listener_->waitForTransform(to_frame.data(),from_frame.data(),ros::Time(),ros::Duration(5.0));
    this->tf_listener_->transformPose(to_frame.data(),from,to);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void WANGBX_ROBOT::Mapper::LidarCallBack(const sensor_msgs::LaserScan &msg)
{
  char temp[1000][1000];
  memset(temp,this->unknown_threshold_,sizeof(temp));
  vector<unsigned char> data;

  this->lidar_.angle_increment = msg.angle_increment;
  this->lidar_.angle_min = msg.angle_min;
  this->lidar_.angle_max = msg.angle_max;
  this->lidar_.range_max = msg.range_min;
  this->lidar_.range_max = msg.range_max;
  this->lidar_.ranges.assign(msg.ranges.begin(),msg.ranges.end());
  this->lidar_.intensities.assign(msg.intensities.begin(),msg.intensities.end());

  double range,angle,c,s,lx,ly;
  int x,y;
  for(int i = 0;i < this->lidar_.ranges.size();i ++)
  {
    range = this->lidar_.ranges.at(i);
    angle = this->lidar_.angle_min + this->lidar_.angle_increment * i;
    c = cos(angle);
    s = sin(angle);
    for(double r = 0.0;r < range + this->map_resolution_;r += this->map_resolution_)
    {
      lx = r*c;
      ly = r*s;
      x = (this->map_height_/2*this->map_resolution_+(lx*c-ly*s))/this->map_resolution_;
      y = (this->map_width_/2*this->map_resolution_+(lx*s+ly*c))/this->map_resolution_;

      temp[x][y] = this->free_threshold_;
    }
    temp[x][y] = this->occ_thrshold_;
  }

  for(int i = 0;i < this->map_height_;i ++)
  {
    for(int j = 0;j < this->map_width_;j ++)
    {
      if(temp[i][j] == this->unknown_threshold_)
      {
        data.push_back(this->unknown_threshold_);
      }
      else if(temp[i][j] == this->free_threshold_)
      {
        data.push_back(this->free_threshold_);
      }
      else if(temp[i][j] == this->occ_thrshold_)
      {
        data.push_back(this->occ_thrshold_);
      }
      else
        ;
    }
  }

  this->lidar_.ranges.clear();
  this->lidar_.intensities.clear();

  this->PublishMap(data);
}

void WANGBX_ROBOT::Mapper::OdomCallBack(const nav_msgs::Odometry &msg)
{
  this->odom_.pose.pose.position.x = msg.pose.pose.position.x;
  this->odom_.pose.pose.position.y = msg.pose.pose.position.y;
  this->odom_.pose.pose.orientation.x = msg.pose.pose.orientation.x;
  this->odom_.pose.pose.orientation.y = msg.pose.pose.orientation.y;
  this->odom_.pose.pose.orientation.z = msg.pose.pose.orientation.z;
  this->odom_.pose.pose.orientation.w = msg.pose.pose.orientation.w;
}

void WANGBX_ROBOT::Mapper::PublishMap(vector<unsigned char> data)
{
  this->map_.header.stamp = ros::Time::now();
  this->map_.header.frame_id = this->map_frame_.data();
  this->map_.info.height = this->map_height_;
  this->map_.info.width = this->map_width_;
  this->map_.info.resolution = this->map_resolution_;
  this->map_.info.origin.position.x = -this->map_height_/2*this->map_resolution_;
  this->map_.info.origin.position.y = -this->map_width_/2*this->map_resolution_;
  this->map_.info.origin.position.z = 0;
  this->map_.info.origin.orientation.x = tf::createQuaternionFromYaw(this->map_yaw_).getX();
  this->map_.info.origin.orientation.y = tf::createQuaternionFromYaw(this->map_yaw_).getY();
  this->map_.info.origin.orientation.z = tf::createQuaternionFromYaw(this->map_yaw_).getZ();
  this->map_.info.origin.orientation.w = tf::createQuaternionFromYaw(this->map_yaw_).getW();

  this->map_.data.assign(data.begin(),data.end());

  this->map_pub_->publish(this->map_);
  this->map_.data.clear();

  geometry_msgs::TransformStamped tfs;
  tfs.header.frame_id = this->map_frame_.data();
  tfs.header.stamp = ros::Time::now();
  tfs.child_frame_id = this->odom_frame_.data();
  tfs.transform.translation.x = 0.0f;
  tfs.transform.translation.y = 0.0f;
  tfs.transform.translation.z = 0.0f;
  tfs.transform.rotation.x = 0.0f;
  tfs.transform.rotation.y = 0.0f;
  tfs.transform.rotation.z = 0.0f;
  tfs.transform.rotation.w = 1.0f;
  this->tf_broadcaster_->sendTransform(tfs);
}
