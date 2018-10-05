#include <multi_robot_sl/config.h>

void mkExperimentDataDir()
{
    cout << "Enter experiment serial number to start PSO:";
    scanf("%d",&experiment_serial_number);
    cout << "Start the experiment" << endl;
    sprintf(file_path,"mkdir -p ./PSO_experiment_data/%d/client_1",experiment_serial_number);
    system(file_path);

    sprintf(file_path,"mkdir -p ./PSO_experiment_data/%d/client_2",experiment_serial_number);
    system(file_path);

    sprintf(file_path,"mkdir -p ./PSO_experiment_data/%d/client_3",experiment_serial_number);
    system(file_path);
       
    sprintf(file_path,"./PSO_experiment_data/%d/client_%d",experiment_serial_number,robot_id);
    cout << "The file path is " << file_path << endl;
}

// void mkExperimentDataDir(int robot_id)
// {
//     ROS_ERROR_STREAM(robot_id);
//     char cmd[1024];
//     FILE *experiment_serial_number_file;

//     sprintf(cmd,"mkdir -p ./PSO_experiment_data");
//     system(cmd);
    
//     switch (robot_id)
//     {
//         case 1:
//             cout << "Enter experiment serial number to start PSO:" << endl;
//             scanf("%d",&experiment_serial_number);
//             experiment_serial_number_file = fopen("./PSO_experiment_data/experiment_serial_number_file","w");
//             fwrite(&experiment_serial_number,sizeof(int),1,experiment_serial_number_file);
//             if(experiment_serial_number_file == NULL)
//             {
//             ROS_ERROR_STREAM("File open error");
//             }
//             fclose(experiment_serial_number_file);
//             break;
//         case 2 :
//         case 3 :
//             experiment_serial_number_file = fopen("./PSO_experiment_data/experiment_serial_number_file","r");
            
//             if(experiment_serial_number_file == NULL)
//             {
//                 ROS_ERROR_STREAM("File open error,try to start with clien_1.");
//             }
//             fread(&experiment_serial_number,sizeof(int),1,experiment_serial_number_file);
//             fclose(experiment_serial_number_file);
//             break;
//         default:
//             ROS_ERROR_STREAM("Error,try to start with clien_1.");
//     }
    


//     sprintf(cmd,"mkdir -p ./PSO_experiment_data/%d/client_1",experiment_serial_number);
//     system(cmd);

//     sprintf(cmd,"mkdir -p ./PSO_experiment_data/%d/client_2",experiment_serial_number);
//     system(cmd);

//     sprintf(cmd,"mkdir -p ./PSO_experiment_data/%d/client_3",experiment_serial_number);
//     system(cmd);
       
//     sprintf(cmd,"./PSO_experiment_data/%d/client_%d",experiment_serial_number,robot_id);
//     cout << "The file path is " << cmd << endl;
// }

bool isNearSource(double x,double y)
{
    //ROS_INFO("x=%f,y=%f",x,y);
    if((fabs(x-X_SOURCE_POSITION)<SOURCE_RANGE)&&(fabs(y-Y_SOURCE_POSITION)<SOURCE_RANGE))
        {
            ROS_INFO("Source~~~~");
            return true;
        }
    else
            return false;
}

void assignGoal_polar(RobotMsgs  array[],int counter,double length,double temp_theta)//极坐标方式给目标点赋值
{
    array[counter+1].position.target_pose.pose.position.x
    = array[counter].position.target_pose.pose.position.x
    + length*cos(temp_theta);

    array[counter+1].position.target_pose.pose.position.y
    = array[counter].position.target_pose.pose.position.y
    + length*sin(temp_theta);

    array[counter+1].position.target_pose.pose.orientation.z
    = sin(temp_theta/2.0);

    array[counter+1].position.target_pose.pose.orientation.w
    = cos(temp_theta/2.0);

    array[counter+1].position.target_pose.header.frame_id = "map";

    array[counter+1].position.target_pose.header.stamp = ros::Time::now();
}

void assignGoal_cartesian(RobotMsgs  array[],int counter,double x,double y)//笛卡尔坐标系方式给目标点赋值
{
    array[counter+1].position.target_pose.pose.position.x
    = x;

    array[counter+1].position.target_pose.pose.position.y
    = y;

    double temp_theta;
    temp_theta = atan(y/x);


    array[counter+1].position.target_pose.pose.orientation.z
    = sin(temp_theta/2.0);

    array[counter+1].position.target_pose.pose.orientation.w
    = cos(temp_theta/2.0);

    array[counter+1].position.target_pose.header.frame_id = "map";

    array[counter+1].position.target_pose.header.stamp = ros::Time::now();
}

void sendGoal(MoveBaseClient& actionlibClient,MoveBaseGoal goal)
{
    
    actionlibClient.sendGoal(goal);
    actionlibClient.waitForResult(ros::Duration(45.0));
    ROS_INFO_STREAM("Heading to ("<<goal.target_pose.pose.position.x<<", "<<goal.target_pose.pose.position.y<<")");
    if(actionlibClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal reached.");
        }
    else
        {
            ROS_INFO("Failed to reach the goal."); 
        }
}

void frw(RobotMsgs  array[],int counter_max,int change_counter,char algorithm_type[])
{
    int counter;
    time_t now;
    struct tm *tm_now ;
    time(&now) ;
    tm_now = localtime(&now) ;
	char str[100];

	sprintf(str,"./PSO_experiment_data/%s-%d-%d-%d-%d-%d-%d.txt"
	        ,algorithm_type,tm_now->tm_year+1900, tm_now->tm_mon+1, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
	ROS_INFO("Saving data to %s",str);

    
    ofstream fout(str,ios::app);
    fout<<"algorithm_type = "<<algorithm_type<<"\n";
    fout<<"robot_id = "<<array[1].robot_id<<"\n";
    fout<<"max counter = "<<counter_max<<"\n";
    fout<<"change counter = "<<change_counter<<"\n";
    fout<<tm_now->tm_year+1900<<tm_now->tm_mon+1<<tm_now->tm_mday<<tm_now->tm_hour<<tm_now->tm_min<<"\n";
    fout<<"counter"<<"\t"<<"x"<<"\t"<<"y"<<"\t"<<"theta"<<"\t"<<"z"<<"\t"<<"w"<<"\t"<<"time"<<"\t"<<"concentration"<<"\t"<<"wind_speed"<<"\t"<<"wind_direction"<<"\n";
    
    
    for(counter=0;counter<counter_max+1;counter++)
    {
        fout<<counter<<"\t"
            <<array[counter].position.target_pose.pose.position.x<<"\t"
            <<array[counter].position.target_pose.pose.position.y<<"\t"
            <<asin(array[counter].position.target_pose.pose.orientation.z)*2<<"\t"
            <<array[counter].position.target_pose.pose.orientation.z<<"\t"
            <<array[counter].position.target_pose.pose.orientation.w<<"\t"
            <<array[counter].position.target_pose.header.stamp<<"\t"
            <<array[counter].alco_concentration.concentration<<"\t"
            <<array[counter].wind_information.speed<<"\t"
            <<array[counter].wind_information.direction<<"\n";
    }

    fout.close();

    ROS_INFO("Data saved.");
}

bool isInRange(double x,double y)
{
    //ROS_INFO("x=%f,y=%f",x,y);
    if((x>X_MIN)&&(x<X_MAX)&&(y>Y_MIN)&&(y<Y_MAX))
        return true;
    else
        {
            ROS_WARN("The goal is not in the range.");
            return false;
        }
}

void obstacleAvoiding_polar(RobotMsgs array[],int counter,double temp_theta)//极坐标避障
{

    double x = array[counter+1].position.target_pose.pose.position.x;
    double y = array[counter+1].position.target_pose.pose.position.y;
    double x0 = array[counter].position.target_pose.pose.position.x;
    double y0 = array[counter].position.target_pose.pose.position.y;
    if(!isInRange(x,y))
    {
        if(( y > Y_MAX ) && ( x < X_MIN || x > X_MAX))   //上面两个角落
            {
               temp_theta =  ( -M_PI + temp_theta );
               x = x + 0.3 * cos(temp_theta);
               y = y + 0.3 * sin(temp_theta);
            }
        if(( y < Y_MIN ) && ( x < X_MIN || x > X_MAX))   //下面面两个角落
            {
               temp_theta =  ( M_PI + temp_theta );
               x = x + 0.3 * cos(temp_theta);
               y = y + 0.3 * sin(temp_theta);
            }                                                                                
        if(( y > Y_MAX ||  y < Y_MIN  ) &&  x > X_MIN && x < X_MAX)   //上下面中间
            {
               temp_theta =  - temp_theta ;
               y = y + 2 * (y0-y);
            }   
        if(( x > X_MAX ||  x < X_MIN  ) &&  y > Y_MIN && y < Y_MAX)   //左右面中间
            {
               temp_theta =  M_PI- temp_theta ;
               x = x + 2 * (x0-x);
            }            

        array[counter+1].position.target_pose.pose.orientation.z
        = sin(temp_theta/2.0);

        array[counter+1].position.target_pose.pose.orientation.w
        = cos(temp_theta/2.0);
    } 
    array[counter+1].position.target_pose.pose.position.x = x;
    array[counter+1].position.target_pose.pose.position.y = y;

    
}

void obstacleAvoiding_cartesian(RobotMsgs array[],int counter)//笛卡尔方式避障
{

    double x = array[counter+1].position.target_pose.pose.position.x;
    double y = array[counter+1].position.target_pose.pose.position.y;
    double x0 = array[counter].position.target_pose.pose.position.x;
    double y0 = array[counter].position.target_pose.pose.position.y;
    double temp_theta = atan((y-y0)/(x-x0));
    if(!isInRange(x,y))
    {
        ROS_ERROR_STREAM("Avoiding Obstacle");
        if(( y > Y_MAX ) && ( x < X_MIN || x > X_MAX))   //上面两个角落
            {
               temp_theta =  ( -M_PI + temp_theta );
               x = x + 0.3 * cos(temp_theta);
               y = y + 0.3 * sin(temp_theta);
            }
        if(( y < Y_MIN ) && ( x < X_MIN || x > X_MAX))   //下面面两个角落
            {
               temp_theta =  ( M_PI + temp_theta );
               x = x + 0.3 * cos(temp_theta);
               y = y + 0.3 * sin(temp_theta);
            }                                                                                
        if(( y > Y_MAX ||  y < Y_MIN  ) &&  x > X_MIN && x < X_MAX)   //上下面中间
            {
               temp_theta =  - temp_theta ;
               y = y + 2 * (y0-y);
            }   
        if(( x > X_MAX ||  x < X_MIN  ) &&  y > Y_MIN && y < Y_MAX)   //左右面中间
            {
               temp_theta =  M_PI- temp_theta ;
               x = x + 2 * (x0-x);
            }
        ROS_ERROR_STREAM("The new goal is:");
        ROS_ERROR_STREAM("x = " << x << ", y = " << y);                 
    } 
    array[counter+1].position.target_pose.pose.position.x = x;
    array[counter+1].position.target_pose.pose.position.y = y;
}



double getMaxInThree(float a, float b,float c)
{
    float max = -1000;
    if (max < a)
        max = a;
    if (max < b)
        max = b;
    if (max < c)
        max = c;
    return max;
}

int getMaxCounterInArray(RobotMsgs a[],int counter)
{
    double max_concentration  = 0;
    int max_counter = 0;
    for(int i = 0; i <= counter ; i++)
        {
            if(max_concentration < a[i].alco_concentration.concentration)
                {
                    max_concentration = a[i].alco_concentration.concentration;
                    max_counter = i;
                }
            else
                continue;
        }
    return max_counter;
}

struct P
{
    double x;
    double y;
};

//为获取全局最优而定义的结构体
struct GO
{
    int client_num;
    int counter;
};

GO getMaxCounterInThreeArray(RobotMsgs client_1[],RobotMsgs client_2[],RobotMsgs client_3[],int counter)
{
    
    int max_counter_1 = getMaxCounterInArray(client_1,counter);
    int max_counter_2 = getMaxCounterInArray(client_2,counter);
    int max_counter_3 = getMaxCounterInArray(client_3,counter);

    int max_counter = max_counter_1;
    int max_client = 1;

    double max_concentration = client_1[max_counter_1].alco_concentration.concentration;
    if (max_concentration < client_2[max_counter_2].alco_concentration.concentration)
        {
            max_client = 2;
            max_counter = max_counter_2;
        }
    else if (max_concentration < client_3[max_counter_3].alco_concentration.concentration)
        {
            max_client = 3;
            max_counter = max_counter_3;
        }

    GO a; 

    a.client_num = max_client;
    a.counter = max_counter;

    return a;
}