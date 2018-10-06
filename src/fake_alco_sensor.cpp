#include <multi_robot_sl/config.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_alco_sensor");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<multi_robot_sl::alco_concentration>("alco_concentration", 1);
  
  //tf部分
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string map_frame="/map";
  std::string base_frame="/base_footprint";//两个frame的名称很重要
  try
  {
    listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(2.0) );
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }


  ros::Rate loop_rate(10);

  multi_robot_sl::alco_concentration fake_alco;
  double x,y;

  while (ros::ok())
  {
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    fake_alco.concentration = 200 - 10 * sqrt(pow( x - X_SOURCE_POSITION, 2)+pow( y - Y_SOURCE_POSITION,2));
    pub.publish(fake_alco);
    loop_rate.sleep();
  }


  return 0;
}