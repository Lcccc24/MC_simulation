/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

bool offboard_enabled = false;
bool armed = false;

//订阅 mavros/state 的回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped TAR_P;
void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    TAR_P = *msg;
}

int main(int argc, char **argv)
{
    //初始化ros系统，节点命名
    ros::init(argc, argv, "offboard_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mother/target_p", 10, target_cb);

    /*
    创建订阅Subscribe；<>里面为模板参数，传入的是订阅的消息体类型，
    ()里面传入三个参数，分别是该消息的位置、缓存大小（通常为10）、回调函数
    订阅 mavros 状态：mavros/state    
    */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/AVC/mavros/state", 10, state_cb);
    /*
    发布之前需要公告，并获取句柄，发布的消息体的类型为：geometry_msgs::PoseStamped
    发布公告 mavros/setpoint_position/local
    */
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/AVC/mavros/setpoint_position/local", 10);
    /*
    启动服务用的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
    启动服务1，设置客户端（Client）名称为arming_client，客户端的类型为ros::ServiceClient: 连接服务 mavros/cmd/arming
    */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/AVC/mavros/cmd/arming");
    /*    
    启动服务2，设置客户端（Client）名称为set_mode_client，客户端的类型为ros::ServiceClient:连接服务 mavros/set_mode
    */
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/AVC/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    //发送频率设置为2hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    // 等待飞控连接mavros，其中current_state是我们订阅的mavros的状态，
    //一直等待 current_state.connected 为 true，表示连接成功，跳出循环。此时已经得到了 mavros/state 消息，飞控已经成功连接 mavros
    while(ros::ok() && !current_state.connected)
    {
        ROS_INFO("connect mavros fail");
        ros::spinOnce();//执行消息回调
        rate.sleep();//睡眠保证频率
    }

    //实例化一个 geometry_msgs::PoseStamped 类，并把 Z 轴赋予 2 米  在下面的循环中由publisher将其发布
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的客户端与服务端之间的通信（服务）
    // 实例化 mavros_msgs::SetMode 类，offb_set_mode.request.custom_mode 赋 值 为“OFFBOARD”，表示我们要进入 OFFBOARD 模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    
    //建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的客户端与服务端之间的通信（服务）
    //实例化 mavros_msgs::CommandBool，arm_cmd.request.value = true;表示我们要解锁解锁 (= true 解锁 = false 加锁)
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //采集当前系统时间
    ros::Time last_request = ros::Time::now();
    
 
    while(ros::ok())
    {  	
    	/*
    	首先判断当前模式是否为offboard模式，如果不是，
    	则客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
        */
    	//如果模式还不为 offboard，并且距离上一次执行进入 offboard 命令(或者系统首次运行)已经过去了 5秒，进入这里
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.1)) && !offboard_enabled)
        {
            //判断是否请求服务成功且服务返回执行成功
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");//打开模式，打印消息 "Offboard enabled"
                offboard_enabled = true;
            }
            else
            {
                ROS_INFO("OFFBOARD FAIL");
            }
            //更新当前时间
            last_request = ros::Time::now();
        } 
        
        //如果已经处于 offboard 模式进入这里
        else 
        {
            /*
            若已经为offboard模式，则判断飞机是否解锁，如果没有解锁，
            则客户端arming_client向服务端arm_cmd发起call请求；然后服务端回应response成功解锁 
            */
            //如果还未解锁，并且距离上次此请求解锁已经过去 5 秒，进入这里
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.1)) && !armed)
            {
                //判断是否请求服务成功且服务返回执行成功 
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");//打印消息 "Vehicle armed"
                    armed = true;
                }
                else{
                    ROS_INFO("ARM FAIL");
                }
                //更新当前时间
                last_request = ros::Time::now();
            }
        }
        //发布位置消息
        local_pos_pub.publish(TAR_P);

        //消息回调保持频率
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
