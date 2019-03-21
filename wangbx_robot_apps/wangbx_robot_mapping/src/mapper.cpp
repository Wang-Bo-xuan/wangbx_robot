#include <ros/ros.h>
#include <wangbx_robot_mapping/wangbx_robot_mapper.h>

int main(int argc,char *argv[])
{
  ros::init(argc,argv,"wangbx_robot_mapping_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1.0);

  WANGBX_ROBOT::Mapper mapper(&nh);

  while(ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
