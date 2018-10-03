#include <multi_robot_sl/config.h>

multi_robot_sl::robot_msgs msgs_temp;

void wind_subCallback(const multi_robot_sl::wind_information &msg)
{
  msgs_temp.wind_information = msg;
}

void alco_subCallback(const multi_robot_sl::alco_concentration &msg)
{
  msgs_temp.alco_concentration = msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_msgs_publisher"); 
    ros::NodeHandle nh;
    
    //订阅传感器
    ros::Subscriber wind_sub = nh.subscribe("wind_information", 1, wind_subCallback);
    ros::Subscriber alco_sub = nh.subscribe("alco_concentration", 1, alco_subCallback);
    
    ros::Publisher  pub = nh.advertise<RobotMsgs>("robot_msgs",1);
    
    nh.getParam("robot_id",msgs_temp.robot_id);

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
    while(ros::ok())
    {
        listener.lookupTransform(map_frame,base_frame, ros::Time(0), transform);
        msgs_temp.position.target_pose.header.frame_id = "map";
        msgs_temp.position.target_pose.header.stamp = ros::Time::now();
        msgs_temp.position.target_pose.pose.position.x = transform.getOrigin().x(); 
        msgs_temp.position.target_pose.pose.position.y = transform.getOrigin().y();
        msgs_temp.position.target_pose.pose.orientation.z = transform.getRotation().z();
        msgs_temp.position.target_pose.pose.orientation.w = transform.getRotation().w(); 

        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        
        pub.publish(msgs_temp);
    }
}


