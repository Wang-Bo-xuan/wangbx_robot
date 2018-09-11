#include <ros/ros.h>
#include "qt_ui.h"

using namespace WANGBX_ROBOT;
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"wangbx_robot_qt_teleop_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);

  teleop_UI teleoper(argc,argv);

  while(ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
