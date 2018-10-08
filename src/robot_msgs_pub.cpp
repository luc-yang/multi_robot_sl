#include <multi_robot_sl/config.h>

multi_robot_sl::robot_msgs msgs_temp;
multi_robot_sl::wind_information local_wind;

void wind_subCallback(const multi_robot_sl::wind_information &msg)
{
  local_wind = msg;
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
    
    double x,y,z,w;
    double yaw,pitch,roll;
    double global_wind_direction;
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        listener.lookupTransform(map_frame,base_frame, ros::Time(0), transform);
        
        //四元数转换，获取精确的弧度制yaw,这个就是机器人的朝向
        x = transform.getRotation().x(); 
        y = transform.getRotation().y();
        z = transform.getRotation().z();
        w = transform.getRotation().w(); 
        tf::Quaternion q(x,y,z,w);
        tf::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);

        msgs_temp.position.target_pose.header.frame_id = "map";
        msgs_temp.position.target_pose.header.stamp = ros::Time::now();
        msgs_temp.position.target_pose.pose.position.x = transform.getOrigin().x(); 
        msgs_temp.position.target_pose.pose.position.y = transform.getOrigin().y();
        msgs_temp.position.target_pose.pose.orientation.z = transform.getRotation().z();
        msgs_temp.position.target_pose.pose.orientation.w = transform.getRotation().w(); 
        msgs_temp.yaw = yaw;

        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();

        //这里生成的风向指风的源头所在的位置的全局角度。
        msgs_temp.wind_information.speed = local_wind.speed;
        global_wind_direction = local_wind.direction + yaw;
        if(global_wind_direction <= - 1.2 * M_PI)
            msgs_temp.wind_information.direction = fabs(global_wind_direction) - M_PI;
        else
            msgs_temp.wind_information.direction = global_wind_direction;
        pub.publish(msgs_temp);
        loop_rate.sleep();
    }
}


