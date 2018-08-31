#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <termio.h>
#include <stdio.h>

int main(int argc,char *argv[])
{
  ros::init(argc,argv,"keyboard_teleop_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  double cur_tran_vel = 0.2;
  double cur_rot_vel = 1.2;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0f;
  vel.angular.z = 0.0f;

  printf("\nControl Your Wangbx_Robot!\n");
  printf("--------------------------------\n");
  printf("Moving around:\n");
  printf("  u  i  o\n");
  printf("  j  k  l\n");
  printf("  m  ,  .\n");
  printf("\n");
  printf("q/z : increase/decrease max speeds by 10%\n");
  printf("w/x : increase/decrease only linear speed by 10%\n");
  printf("e/c : increase/decrease only angular speed by 10%\n");
  printf("\n");
  printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);

  struct termios new_settings,stored_settings;

  while(ros::ok())
  {
    char in;

    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    in = getchar();
    tcsetattr(0,TCSANOW,&stored_settings);

    switch(in)
    {
      case 'i':
        vel.linear.x = cur_tran_vel;
        vel.angular.z = 0.0f;
        vel_pub.publish(vel);
      break;
      case ',':
        vel.linear.x = -cur_tran_vel;
        vel.angular.z = 0.0f;
        vel_pub.publish(vel);
      break;
      case 'j':
        vel.linear.x = 0.0f;
        vel.angular.z = cur_rot_vel;
        vel_pub.publish(vel);
      break;
      case 'l':
        vel.linear.x = 0.0f;
        vel.angular.z = -cur_rot_vel;
        vel_pub.publish(vel);
      break;
      case 'k':
        vel.linear.x = 0.0f;
        vel.angular.z = 0.0f;
        vel_pub.publish(vel);
      break;
      case 'q':
        cur_tran_vel *= 1.1;
        cur_rot_vel *= 1.1;
        printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);
      break;
      case 'z':
        cur_tran_vel /= 1.1;
        cur_rot_vel /= 1.1;
        printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);
      break;
      case 'w':
        cur_tran_vel *= 1.1;
        printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);
      break;
      case 'x':
        cur_tran_vel /= 1.1;
        printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);
      break;
      case 'e':
        cur_rot_vel *= 1.1;
        printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);
      break;
      case 'c':
        cur_rot_vel /= 1.1;
        printf("currently:\tspeed:%lf\tturn:%lf\n",cur_tran_vel,cur_rot_vel);
      break;
      default:
        vel.linear.x = 0.0f;
        vel.angular.z = 0.0f;
        vel_pub.publish(vel);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

	return 0;
}
