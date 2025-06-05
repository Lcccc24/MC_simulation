/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>

#define VELOCITY2D_CONTROL 0b011111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE,设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
#define POSITION_CONTROL 0b011111111000 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE

mavros_msgs::State current_state;
std_msgs::Int32 move_cmd;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void move_cb(const std_msgs::Int32::ConstPtr& msg){
    move_cmd = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mother_hover");
    ros::NodeHandle nh;

    //订阅mavros状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/AVC/mavros/state", 10, state_cb);

    ros::Subscriber move_sub = nh.subscribe<std_msgs::Int32>
            ("/mother_move/cmd", 1, move_cb);

    //发布无人机位姿信息
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/AVC/mavros/setpoint_raw/local", 10);
    //定义起飞服务客户端（起飞，降落）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/AVC/mavros/cmd/arming");
    //定义设置模式服务客户端（设置offboard模式）
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/AVC/mavros/set_mode");



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    double xy_amplitude_, z_amplitude_, frequency_;

    nh.param("xy_amplitude", xy_amplitude_, 0.1);
    nh.param("z_amplitude", z_amplitude_, 0.1);
    nh.param("frequency", frequency_, 0.1);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget goal;
    goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    goal.header.stamp = ros::Time::now();
    goal.type_mask = POSITION_CONTROL;
    goal.position.x = 1.0;
    goal.position.y = 2.0;
    goal.position.z = 1.5;

    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(goal);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        //ROS_INFO("mother hovering");

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        else {
            if( !current_state.armed &&(ros::Time::now() - last_request > ros::Duration(0.5))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(move_cmd.data == 1){
            goal.position.x = 3.0;
            goal.position.y = 5.0;
            goal.position.z = 2.5;
        }

        else{
            goal.position.x = 1.0;
            goal.position.y = 2.0;
            goal.position.z = 1.5;
        }

        // 计算经过的时间（秒）
        double t = (ros::Time::now() - last_request).toSec();
        
        // XY振荡（原有逻辑）
        double dx = xy_amplitude_ * sin(2 * M_PI * frequency_ * t);
        double dy = xy_amplitude_ * cos(2 * M_PI * frequency_ * t);
        
        // Z轴振荡（新增部分）
        double dz = z_amplitude_ * sin(2 * M_PI * frequency_ * t + M_PI/2); // 相位差90度
        
        goal.position.x += dx;
        goal.position.y += dy;
        goal.position.z += dz; // 添加Z轴振荡

        local_pos_pub.publish(goal);
        
        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}
