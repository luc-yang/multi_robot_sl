#include <multi_robot_sl/config.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_alco_sensor");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<multi_robot_sl::alco_concentration>("alco_concentration", 1);
  

  ros::Rate loop_rate(10);

  multi_robot_sl::alco_concentration fake_alco;
  fake_alco.concentration = 99;

  while (ros::ok())
  {
    pub.publish(fake_alco);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}