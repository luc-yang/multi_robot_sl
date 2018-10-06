#include "ros/ros.h"
#include "std_msgs/String.h"
#include <multi_robot_sl/functions.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "for_test");

  ros::NodeHandle n;

  

  

  while (ros::ok())
  {
    system("xdotool key ctrl+shift+o");
    ros::Duration(1.0).sleep();//休眠一秒
    system("xdotool type 'echo $ROS_MASTER_URI\n'");
    ros::Duration(1.0).sleep();//休眠一秒
  }
  


  return 0;
}