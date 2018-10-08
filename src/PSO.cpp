#include <cstdlib> //文件读写用到的头文件
#include <multi_robot_sl/functions.h>



char algorithm_type[] = "PSO";
int state = 0;// state代表机器人所处的状态，然后在下面就可以用swich来跳转状态

RobotMsgs temp_robot_msgs;
RobotMsgs array_robot_msgs_host[ARRAY_LENGTH];

OPTIMUM global_optimum[ARRAY_LENGTH];
OPTIMUM local_optimum[ARRAY_LENGTH];

void robot_msgs_subCallback(const RobotMsgs &msg)
{
    temp_robot_msgs = msg;
}

void getRobotMsgs(RobotMsgs &msg)//输入robot_msgs_array[counter]，会修改这个值。得到当前的平均浓度和风向
{
    float concentration_temp = 0;
    float wind_speed_temp = 0;
    float wind_direction_temp = 0;
    
    for(int s_i=0;s_i < record_seconds; s_i++)
    {
        ros::spinOnce();
        msg = temp_robot_msgs;
        if (s_i+1 == 1)
            ROS_INFO_STREAM(s_i + 1 <<" second recorded.");
        else
            ROS_INFO_STREAM(s_i + 1 <<" seconds recorded.");
        if(s_i>wait_seconds-1)//前五秒的数据丢掉,避免机器人本身移动对气流的影响
        {
            concentration_temp  += temp_robot_msgs.alco_concentration.concentration;
            wind_speed_temp     += temp_robot_msgs.wind_information.speed;
            wind_direction_temp += temp_robot_msgs.wind_information.direction;
        }

        ros::Duration(1.0).sleep();//休眠一秒
    }
    //data = robot_msgs_temp;//坐标等信息被传入
    //注释上一行，为了使用HEX-PATH。也就是说除了第一次是采取实时坐标，其他时间都用根据第一个坐标推算出来的六边形的坐标
    msg.alco_concentration.concentration   = concentration_temp/(record_seconds-wait_seconds);
    msg.wind_information.speed             = wind_speed_temp/(record_seconds-wait_seconds);
    msg.wind_information.direction         = wind_direction_temp/(record_seconds-wait_seconds);
    ROS_ERROR_STREAM("alco_concentration\twind_speed\twind_direction");
    ROS_ERROR_STREAM(msg.alco_concentration.concentration << "\t" <<msg.wind_information.speed<< "\t" <<msg.wind_information.direction);
}

int main(int argc, char *argv[])
{
    //初始化节点
    ros::init(argc, argv, "PSO"); 

    //定义传感器订阅
    ros::NodeHandle nh;

    ros::Subscriber sub_robot_msgs = nh.subscribe("robot_msgs", 1, robot_msgs_subCallback);

    ros::Rate loop_rate(10);
    
    int counter = 0;//机器人行进的步数
    int change_counter = 7; //发现改变策略的步数


    //自动生成路径
    nh.getParam("robot_id", robot_id);
    ROS_INFO_STREAM("Robot id = " << robot_id);
    mkExperimentDataDir();

    int counter_1,counter_2,counter_3;
    counter_1 = int(rand());
    counter_2 = int(rand());
    counter_3 = int(rand());
    FILE *client_1_counter;
    FILE *client_2_counter;
    FILE *client_3_counter;
    FILE *robot_msgs_client_1;
    FILE *robot_msgs_client_2;
    FILE *robot_msgs_client_3;





    // 记得先给三个counter文件赋随机整数值，否则容易出现段错误，主要是因为counter过早一致，会让某个机器人读还没有产生的值
    client_1_counter = fopen("./PSO_experiment_data/client_1_counter","w");
    fwrite(&counter_1,sizeof(int),1,client_1_counter);
    if(client_1_counter == NULL)
    {
    ROS_ERROR_STREAM("File open error");
    }
    fclose(client_1_counter);

    client_2_counter = fopen("./PSO_experiment_data/client_2_counter","w");
    fwrite(&counter_2,sizeof(int),1,client_2_counter);
    if(client_2_counter == NULL)
    {
    ROS_ERROR_STREAM("File open error");
    }
    fclose(client_2_counter);

    client_3_counter = fopen("./PSO_experiment_data/client_3_counter","w");
    fwrite(&counter_3,sizeof(int),1,client_3_counter);
    if(client_3_counter == NULL)
    {
    ROS_ERROR_STREAM("File open error");
    }
    fclose(client_3_counter);

    /*   move base 段  */
    // 定义actionlib的client
    MoveBaseClient ac("move_base", true);
    //等待action server响应
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    /*   move base 段  */



    for(counter = 1; ros::ok() && (counter < 25); counter++)
    {
        
        //获取浓度,位置、风速等信息
        getRobotMsgs(array_robot_msgs_host[counter]);

        array_robot_msgs_host[counter].robot_id = robot_id;
        // ROS_ERROR_STREAM(array_robot_msgs_host[counter]);

        //把信息写入文件
        switch (robot_id)
        {
            case 1:
                sprintf(file_path,"./PSO_experiment_data/%d/client_1/%d",experiment_serial_number,counter);
                robot_msgs_client_1 = fopen(file_path,"w");
                fwrite(&array_robot_msgs_host[counter],sizeof(array_robot_msgs_host[counter]),1,robot_msgs_client_1);
                fclose(robot_msgs_client_1);
                break;
            case 2:
                sprintf(file_path,"./PSO_experiment_data/%d/client_2/%d",experiment_serial_number,counter);
                robot_msgs_client_2 = fopen(file_path,"w");
                fwrite(&array_robot_msgs_host[counter],sizeof(array_robot_msgs_host[counter]),1,robot_msgs_client_2);
                fclose(robot_msgs_client_2);
                break;      
            case 3:
                sprintf(file_path,"./PSO_experiment_data/%d/client_3/%d",experiment_serial_number,counter);
                robot_msgs_client_3 = fopen(file_path,"w");
                fwrite(&array_robot_msgs_host[counter],sizeof(array_robot_msgs_host[counter]),1,robot_msgs_client_3);
                fclose(robot_msgs_client_3);
                break;                        
            default:
                ROS_ERROR_STREAM("client is " << robot_id );
        }

        ROS_DEBUG_STREAM("Robot msg has been written into " << file_path);

        //写counter
        switch (robot_id)
        {
            case 1:
                client_1_counter = fopen("./PSO_experiment_data/client_1_counter","w");
                fwrite(&counter,sizeof(int),1,client_1_counter);
                if(client_1_counter == NULL)
                {
                    ROS_ERROR_STREAM("File open error");
                }
                fclose(client_1_counter);
                break;
            case 2:
                client_2_counter = fopen("./PSO_experiment_data/client_2_counter","w");
                if(client_2_counter == NULL)
                {
                    ROS_ERROR_STREAM("File open error");
                }                
                fwrite(&counter,sizeof(int),1,client_2_counter);
                fclose(client_2_counter);
                break;
            case 3:
                client_3_counter = fopen("./PSO_experiment_data/client_3_counter","w");
                if(client_3_counter == NULL)
                {
                    ROS_ERROR_STREAM("File open error");
                }            
                fwrite(&counter,sizeof(int),1,client_3_counter);
                fclose(client_3_counter);
                break;
            default:
                ROS_ERROR_STREAM("client is " << robot_id );
        }
        ROS_DEBUG_STREAM("Counter has been written into " << file_path);


        //读counter
        client_1_counter = fopen("./PSO_experiment_data/client_1_counter","r");
        if(client_1_counter == NULL)
        {
            ROS_ERROR_STREAM("client_1_counter open error");
        }
        fread(&counter_1,sizeof(int),1,client_1_counter);
        fclose(client_1_counter);
        ROS_DEBUG_STREAM("Counter has been read form client_1_counter");

        client_2_counter = fopen("./PSO_experiment_data/client_2_counter","r");
        if(client_2_counter == NULL)
        {
            ROS_ERROR_STREAM("client_2_counter open error");
        }
        fread(&counter_2,sizeof(int),1,client_2_counter);
        fclose(client_2_counter);
        ROS_DEBUG_STREAM("Counter has been read form client_2_counter");

        // client_3_counter = fopen("./PSO_experiment_data/client_3_counter","r");
        // if(client_2_counter == NULL)
        // {
        //     ROS_ERROR_STREAM("client_3_counter open error");
        // }
        // fread(&counter_3,sizeof(int),1,client_3_counter);
        // fclose(client_3_counter);
        // ROS_DEBUG_STREAM("Counter has been read form client_3_counter");
        
        // 当三个机器人的counter不一样时，就一直等待。
        // while((counter_1 != counter_2) || (counter_1 != counter_3) || (counter_3 != counter_2))
        while((counter_1 != counter_2))
        {
            client_1_counter = fopen("./PSO_experiment_data/client_1_counter","r");
            if(client_1_counter == NULL)
            {
                ROS_ERROR_STREAM("client_1_counter open error");
            }
            fread(&counter_1,sizeof(int),1,client_1_counter);
            fclose(client_1_counter);

            client_2_counter = fopen("./PSO_experiment_data/client_2_counter","r");
            if(client_2_counter == NULL)
            {
                ROS_ERROR_STREAM("client_2_counter open error");
            }
            fread(&counter_2,sizeof(int),1,client_2_counter);
            fclose(client_2_counter);

            // client_3_counter = fopen("./PSO_experiment_data/client_3_counter","r");
            // if(client_2_counter == NULL)
            // {
            //     ROS_ERROR_STREAM("client_3_counter open error");
            // }
            // fread(&counter_3,sizeof(int),1,client_3_counter);
            // fclose(client_3_counter);
            ROS_INFO_STREAM("Waiting:\n " << counter_1 << " " << counter_2 << " " << counter_3);
            ros::Duration(0.3).sleep();
        }


        // 读取消息进数组
        sprintf(file_path,"./PSO_experiment_data/%d/client_1/%d",experiment_serial_number,counter);
        robot_msgs_client_1 = fopen(file_path,"r");
        fread(&array_robot_msgs_client_1[counter],sizeof(array_robot_msgs_host[counter]),1,robot_msgs_client_1);
        // ROS_DEBUG_STREAM(array_robot_msgs_client_1[counter]);
        fclose(robot_msgs_client_1);
        ROS_DEBUG_STREAM("Robot msg has been read from " << file_path);

        sprintf(file_path,"./PSO_experiment_data/%d/client_2/%d",experiment_serial_number,counter);
        robot_msgs_client_2 = fopen(file_path,"r");
        fread(&array_robot_msgs_client_2[counter],sizeof(array_robot_msgs_host[counter]),1,robot_msgs_client_2);
        // ROS_DEBUG_STREAM(array_robot_msgs_client_2[counter]);
        fclose(robot_msgs_client_2);
        ROS_DEBUG_STREAM("Robot msg has been read from " << file_path);

        // sprintf(file_path,"./PSO_experiment_data/%d/client_3/%d",experiment_serial_number,counter);
        // robot_msgs_client_3 = fopen(file_path,"r");
        // fread(&array_robot_msgs_client_3[counter],sizeof(array_robot_msgs_host[counter]),1,robot_msgs_client_3);
        // ROS_DEBUG_STREAM(array_robot_msgs_client_3[counter]);
        // fclose(robot_msgs_client_3);
        // ROS_DEBUG_STREAM("Robot msg has been read from " << file_path);

        // 步数太少则必须继续烟羽发现过程
        if (counter < 5)
            state = 0;
        else
            state = 1;
        
        switch (state)
        {
            case 0:// 烟羽发现
                {
                    double max_in_three = getMaxInThree(    array_robot_msgs_client_1[counter].alco_concentration.concentration,
                                                            array_robot_msgs_client_2[counter].alco_concentration.concentration,
                                                            array_robot_msgs_client_3[counter].alco_concentration.concentration
                                                        );

                    if( max_in_three < ALCO_THRESHOLD )
                        {
                            double current_direction = array_robot_msgs_host[counter].yaw;
                            double step_length = 0.5;

                            assignGoal_polar(array_robot_msgs_host, counter, step_length, current_direction);//0.5是步长，current_direction是全局角度(弧度制)。角度为0的时候，就会向场地正前方走

                            ROS_ERROR_STREAM("origin direction : " << array_robot_msgs_host[counter+1].position.target_pose.pose.orientation.w);

                            obstacleAvoiding_polar(array_robot_msgs_host, counter, current_direction);//这里应该和上面的角度保持一致，代表机器人下一步要前进的角度   

                            ROS_ERROR_STREAM("current direction : " << array_robot_msgs_host[counter+1].position.target_pose.pose.orientation.w);

                            sendGoal(ac,array_robot_msgs_host[counter+1].position);
                            break;
                        }
                    else
                        {
                            state = 1;
                            change_counter = counter;
                            break;
                        }
                }
            case 1:// PSO
                {
                    P p_next,p_cur,p_lo,p_go,p_last,v_wind;
                    //分别是下一步位置，当前位置，局部最优，全局最优,上一步的位置,风向量项

                    //获取局部最优
                    int lo_counter;//局部最优的步数
                    lo_counter = getMaxCounterInArray(array_robot_msgs_host,counter);
                    p_lo.x = array_robot_msgs_host[lo_counter].POSITON.x;
                    p_lo.y = array_robot_msgs_host[lo_counter].POSITON.y;
                    local_optimum[counter].position = p_lo;
                    local_optimum[counter].concentration = array_robot_msgs_host[lo_counter].alco_concentration.concentration;

                    //获取全局最优
                    GO go;
                    go = getMaxCounterInThreeArray(array_robot_msgs_client_1,array_robot_msgs_client_2,array_robot_msgs_client_3,counter);
                    
                    switch(go.client_num)
                    {
                        case 1:
                                {
                                    p_go.x = array_robot_msgs_client_1[go.counter].POSITON.x;
                                    p_go.y = array_robot_msgs_client_1[go.counter].POSITON.y;
                                    global_optimum[counter].concentration = array_robot_msgs_client_1[go.counter].alco_concentration.concentration;
                                    break;
                                }
                        case 2:
                                {
                                    p_go.x = array_robot_msgs_client_2[go.counter].POSITON.x;
                                    p_go.y = array_robot_msgs_client_2[go.counter].POSITON.y;
                                    global_optimum[counter].concentration = array_robot_msgs_client_2[go.counter].alco_concentration.concentration;
                                    break;
                                }
                        case 3:
                                {
                                    p_go.x = array_robot_msgs_client_3[go.counter].POSITON.x;
                                    p_go.y = array_robot_msgs_client_3[go.counter].POSITON.y;
                                    global_optimum[counter].concentration = array_robot_msgs_client_3[go.counter].alco_concentration.concentration;
                                    break;
                                }
                    }
                    global_optimum[counter].position = p_go;
                    global_optimum[counter].go = go;
                    ROS_INFO_STREAM("Global Optimum :client_" << go.client_num << " counter " << go.counter);
                    ROS_INFO_STREAM("GO X = " << p_go.x << ", GO Y = "  << p_go.y );
                    ROS_INFO_STREAM("Local Optimum :client_" << robot_id << " counter " << lo_counter);
                    ROS_INFO_STREAM("LO X = " << p_lo.x << ", LO Y = "  << p_lo.y );

                    // 获取当前位置
                    p_cur.x = array_robot_msgs_host[counter].POSITON.x;
                    p_cur.y = array_robot_msgs_host[counter].POSITON.y;
                    
                    //获取上一步位置
                    p_last.x = array_robot_msgs_host[counter-1].POSITON.x;
                    p_last.y = array_robot_msgs_host[counter-1].POSITON.y;


                    //生成风速度项
                    double wind_speed = array_robot_msgs_host[counter].wind_information.speed;
                    //生成的风向是机器人与风源连线形成向量在世界坐标系下的角度。
                    if(wind_speed < WIND_THRESHOLD)
                    {
                        double temp_angle = M_PI * ((double(rand()%100)/100) * 2 - 1);//随机生成一个-PI到PI的角
                        v_wind.x = STEP_LEN_MAX * cos(temp_angle);//vector_wind  风向量
                        v_wind.y = STEP_LEN_MAX * sin(temp_angle);
                    }
                    else
                    {
                        double wind_direction = array_robot_msgs_host[counter].wind_information.direction;
                        v_wind.x = STEP_LEN_MAX * cos(wind_direction);
                        v_wind.y = STEP_LEN_MAX * sin(wind_direction);
                    }


                    //生成两个随机数
                    double K_1,K_2,K_3;
                    srand(time(NULL)+rand());
                    K_1 = 2 * (double(rand()%100)/100);
                    srand(time(NULL)+rand());
                    K_2 = 2 * (double(rand()%100)/100);
                    srand(time(NULL)+rand());
                    K_3 = 2 * (double(rand()%100)/100);
                    

                    //运算下一位置
                    double delta_x,delta_y;
                    delta_x = p_cur.x - p_last.x + K_1 * (p_lo.x - p_cur.x) + K_2 * (p_go.x - p_cur.x) + K_3 * v_wind.x;
                    delta_y = p_cur.y - p_last.y + K_1 * (p_lo.y - p_cur.y) + K_2 * (p_go.y - p_cur.y) + K_3 * v_wind.x;

                    //计算需要的步长
                    double step_length = sqrt(pow(delta_x,2)+pow(delta_y,2));
                    double direction = atan(delta_y/delta_x);

                    ROS_ERROR_STREAM("step_length = "<<step_length);

                    //对步长进行修正
                    if(step_length > STEP_LEN_MAX)
                    {
                        delta_x = (delta_x) * STEP_LEN_MAX / step_length ;
                        delta_y = (delta_y) * STEP_LEN_MAX / step_length ;
                    }
                    else if((step_length < STEP_LEN_MIN) && (step_length > 0))
                    {
                        delta_x = (delta_x) * (STEP_LEN_MIN + 0.05)/ step_length ;
                        delta_y = (delta_y) * (STEP_LEN_MIN + 0.05) / step_length ;
                    }
                    
                    //获得下一点的坐标
                    p_next.x = p_cur.x + delta_x;
                    p_next.y = p_cur.y + delta_y;
                    ROS_ERROR_STREAM("p_next (" << p_next.x << "," << p_next.y <<")");
                    
                    //更新step_length的值
                    step_length = sqrt(pow(delta_x,2)+pow(delta_y,2));
                    ROS_ERROR_STREAM("new_step_length = "<<step_length);
                    
                    //目标点赋值（直角坐标方式），避障
                    assignGoal_cartesian(array_robot_msgs_host, counter, p_next.x, p_next.y);//0.5是步长，current_direction是全局角度(弧度制)。角度为0的时候，就会向场地正前方走
                    // ROS_ERROR_STREAM("origin direction : " << array_robot_msgs_host[counter+1].position.target_pose.pose.orientation.w);
                    obstacleAvoiding_cartesian(array_robot_msgs_host, counter);
                    
                    //如果执行了避障，则array_robot_msgs_host里的下一点目标就被修改了，所以要再给p_next赋值
                    p_next.x = array_robot_msgs_host[counter+1].position.target_pose.pose.position.x;
                    p_next.y = array_robot_msgs_host[counter+1].position.target_pose.pose.position.y;

                    p_next = psoObstacleAvoiding(robot_id,go,p_cur,p_next);

                    delta_x = p_next.x - p_cur.x;
                    delta_y = p_next.y - p_cur.y;

                    //计算出正确的方向，避免出现机器人重复摆动的情况
                    direction = atan(delta_y/delta_x);
                    array_robot_msgs_host[counter+1].position.target_pose.pose.orientation.w = cos(direction/2.0);
                    array_robot_msgs_host[counter+1].position.target_pose.pose.orientation.z = sin(direction/2.0);

                    // ROS_ERROR_STREAM("current direction : " << array_robot_msgs_host[counter+1].position.target_pose.pose.orientation.w);
                    sendGoal(ac,array_robot_msgs_host[counter+1].position);
                }
        }
        
        int a = 3;
        if ((counter >= change_counter + a) && (state == 1))// 判断是否已经找到源 
        {
            ROS_ERROR_STREAM("Confirming source");
            double x1 = global_optimum[counter].position.x;
            double y1 = global_optimum[counter].position.y;
            double x2 = global_optimum[counter-a+1].position.x;
            double y2 = global_optimum[counter-a+1].position.y;

            double distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));

            if (distance <= 0.1)
            {
                if  (isNearSource(x1,y1))
                    ROS_ERROR_STREAM("SUCCESSED");
                else
                    ROS_ERROR_STREAM("FAILED");
                break;
            }
        }
    }
    ROS_ERROR_STREAM("Stoping PSO.....");
    frw(array_robot_msgs_host,global_optimum,counter, change_counter,algorithm_type);
    
}