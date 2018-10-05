#include <string.h>
#include <fstream>
#include <cmath>
#include <ctime>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <vector>
//标准库文件

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//ROS系统的一些文件

#include <multi_robot_sl/robot_msgs.h>
#include <multi_robot_sl/alco_concentration.h>
#include <multi_robot_sl/wind_information.h>
//自定义的一些消息文件


#define X_MIN -0.5
#define X_MAX 4.0

#define Y_MIN -1.6
#define Y_MAX 1.8

#define SEARCHING_TIME 30

#define X_SOURCE_POSITION 3.45
#define Y_SOURCE_POSITION -0.8
//地图扫描后的实验场地参数

#define ARRAY_LENGTH 150
#define POSITON position.target_pose.pose.position

#define SOURCE_RANGE 0.3

#define STEP_LEN_MAX 0.2
#define STEP_LEN_MIN 0.1
#define WIND_THRESHOLD 1
#define ALCO_THRESHOLD 500

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;
typedef multi_robot_sl::robot_msgs RobotMsgs;

int record_seconds = 2;//record_seconds要大于wait_seconds
int wait_seconds = 0;

int experiment_serial_number;
char file_path[1024];
int robot_id;


RobotMsgs robot_msgs_temp;

RobotMsgs robot_msgs_client_1;
RobotMsgs temp_robot_msgs_client_1;
RobotMsgs array_robot_msgs_client_1[ARRAY_LENGTH];

RobotMsgs robot_msgs_client_2;
RobotMsgs temp_robot_msgs_client_2;
RobotMsgs array_robot_msgs_client_2[ARRAY_LENGTH];

RobotMsgs robot_msgs_client_3;
RobotMsgs temp_robot_msgs_client_3;
RobotMsgs array_robot_msgs_client_3[ARRAY_LENGTH];