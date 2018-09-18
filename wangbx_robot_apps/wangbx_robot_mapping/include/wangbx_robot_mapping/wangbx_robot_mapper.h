#ifndef __INCLUDE_WANGBX_ROBOT_MAPPING_WANGBX_ROBOT_MAPPER_H__
#define __INCLUDE_WANGBX_ROBOT_MAPPING_WANGBX_ROBOT_MAPPER_H__

#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
using namespace std;

namespace WANGBX_ROBOT
{
  class Mapper
  {
    public:
      Mapper(ros::NodeHandle *nh);
      ~Mapper();

    private:
      void Init(void);
      void GetParam(void);
      void TF(double x,double y,double yaw,string form_frame,string to_frame);
      void LidarCallBack(const sensor_msgs::LaserScan &msg);
      void OdomCallBack(const nav_msgs::Odometry &msg);
      void PublishMap(vector<unsigned char> map);

      ros::Subscriber *lidar_sub_;
      ros::Subscriber *odom_sub_;
      ros::Publisher *map_pub_;
      tf::TransformBroadcaster *tf_broadcaster_;
      tf::TransformListener *tf_listener_;

      sensor_msgs::LaserScan lidar_;
      nav_msgs::Odometry odom_;
      nav_msgs::OccupancyGrid map_;
      char temp[1000][1000];

      int map_height_;
      int map_width_;
      double map_yaw_;
      double map_resolution_;
      string map_frame_;
      string odom_frame_;
      int occ_thrshold_;
      int free_threshold_;
      int unknown_threshold_;
  };
}

#endif
