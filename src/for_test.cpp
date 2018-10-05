#include "ros/ros.h"
#include "std_msgs/String.h"
#include <multi_robot_sl/functions.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "for_test");

  ros::NodeHandle n;

  

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
      mkExperimentDataDir();
      mkExperimentDataDir();
      mkExperimentDataDir();
  }
  



  return 0;
}