#include <ros/ros.h> 
#include <serial/serial.h>  //ROS的串口包 
#include <std_msgs/String.h> 
#include <multi_robot_sl/wind_information.h>
#include <fstream>

using namespace std;

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    //发布主题 
    ros::Publisher wind_pub = nh.advertise<multi_robot_sl::wind_information>("wind_information", 20);
    //声明串口对象
    serial::Serial ser;
    try 
    {
    //设置串口属性，并打开串口
        ser.setPort("/dev/wind_sensor"); 
        ser.setBaudrate(9600); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    }
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    multi_robot_sl::wind_information wind_information;
    //指定循环的频率 
    ros::Rate loop_rate(10); 
    uint8_t request[8]={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};
    while(ros::ok()) 
    { 
        ser.write(request,8);   //发送串口数据 
        if(ser.available())
        { 
            std_msgs::String result; 
            result.data = ser.read(ser.available());
            int a[8];
            for(int i=0;i<8;i++)
               {
                   a[i]=(unsigned char)result.data[i];
               }                
            wind_information.speed = (float)((a[3]<<8)+a[4])/100;
            double angle;
            angle = (float)((a[5]<<8)+a[6]);
            if(angle >= 180)
                wind_information.direction = - M_PI / 180 * angle + 2 * M_PI;
            else
                wind_information.direction = - angle / 180 * M_PI;
            wind_pub.publish(wind_information);
        }
        loop_rate.sleep(); 
    } 
} 