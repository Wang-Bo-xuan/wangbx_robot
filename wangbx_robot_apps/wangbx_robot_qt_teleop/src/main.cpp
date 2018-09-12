#include <ros/ros.h>
#include "qt_ui.h"

using namespace WANGBX_ROBOT;
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"wangbx_robot_qt_teleop_node");

  teleop_UI teleoper(argc,argv);

  return 0;
}
