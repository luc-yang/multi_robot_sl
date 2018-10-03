#include <multi_robot_sl/config.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_wind_sensor");


  ros::NodeHandle nh;


  ros::Publisher pub = nh.advertise<multi_robot_sl::wind_information>("wind_information", 1);
  

  ros::Rate loop_rate(10);

  multi_robot_sl::wind_information fake_wind;
  fake_wind.direction = 999;
  fake_wind.speed = 999;

  while (ros::ok())
  {
    pub.publish(fake_wind);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}