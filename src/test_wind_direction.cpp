#include <ros/ros.h>
#include <multi_robot_sl/wind_information.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <iostream>
#include <tf/LinearMath/Matrix3x3.h>


multi_robot_sl::wind_information wind;

void wind_subCallback(const multi_robot_sl::wind_information &msg)
{
  wind = msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_wind");
    
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("wind_information", 1, wind_subCallback);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::string map_frame="/odom";
    std::string base_frame="/base_link";//两个frame的名称很重要
    try
    {
        listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(2.0) );
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    double robo_direction,wind_direction;
    ros::Rate loop_rate(2);
    double x,y,z,w;
    double yaw,pitch,roll;
    
    while(ros::ok())
    {
        listener.lookupTransform("/odom","/base_link", ros::Time(0), transform);
        x = transform.getRotation().x(); 
        y = transform.getRotation().y();
        z = transform.getRotation().z();
        w = transform.getRotation().w(); 
        tf::Quaternion q(x,y,z,w);
        tf::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);
        ros::spinOnce();
        if(wind.direction >= 180)
            wind_direction = - M_PI / 180 * wind.direction + 2 * M_PI;
        else
            wind_direction = - wind.direction / 180 * M_PI;
        double global_wind_direction    = robo_direction + wind_direction;

        if(global_wind_direction <= - 1.2 * M_PI)
            global_wind_direction = fabs(global_wind_direction) - M_PI;
        robo_direction = yaw;
        ROS_INFO_STREAM(robo_direction<<" "<<wind_direction << " " << global_wind_direction);
        loop_rate.sleep();
    }
    return 0;
}