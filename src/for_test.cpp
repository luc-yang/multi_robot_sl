#include "ros/ros.h"
#include "std_msgs/String.h"
#include <multi_robot_sl/functions.h>
#include <cstdlib> //文件读写用到的头文件

int main(int argc, char **argv)
{
  ros::init(argc, argv, "for_test",ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  int num = 1;
  int a=0;
  FILE *aaa;
  aaa = fopen("./111","w");
  fwrite(&num,sizeof(int),1,aaa);
  fclose(aaa);
  while (ros::ok())
  {
    aaa = fopen("./111","w");
    fwrite(&a,sizeof(int),1,aaa);
    ros::Duration(1.0).sleep();
    fclose(aaa);
    ROS_INFO_STREAM(a);
  }
  


  return 0;
}