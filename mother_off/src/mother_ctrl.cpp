#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped init_pose, current_pose;
geometry_msgs::PoseStamped target_pose;
bool pose_inited = false;
double target_wp1_x_, target_wp1_y_, target_wp1_z_, target_wp2_x_, target_wp2_y_ , target_wp2_z_;
int standby_time_;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_inited){
        init_pose = *msg;
        pose_inited = true;
    }
    
    else{
        current_pose = *msg;
        //减去初始偏置 (GPS/RTK)
        // current_pose.pose.position.x -= init_pose.pose.position.x;
        // current_pose.pose.position.y -= init_pose.pose.position.y;
        // current_pose.pose.position.z -= init_pose.pose.position.z;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mother_ctrl");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    nh.param("target_wp1_x", target_wp1_x_, 0.0);
    nh.param("target_wp1_y", target_wp1_y_, 0.0);
    nh.param("target_wp1_z", target_wp1_z_, 1.5);
    nh.param("target_wp2_x", target_wp2_x_, -2.0);
    nh.param("target_wp2_y", target_wp2_y_, -2.0);
    nh.param("target_wp2_z", target_wp2_z_, 1.5);
    nh.param("standby_time", standby_time_, 10);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/AVC/mavros/local_position/pose", 10, pose_cb);

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/mother/target_p", 10);

    while(ros::ok()){
        
        while(!pose_inited){
            ROS_INFO("Wait first pose");
            ros::spinOnce();
            rate.sleep();
        }

        //搭载子机起飞
        target_pose.pose.position.x = target_wp1_x_ + init_pose.pose.position.x;
        target_pose.pose.position.y = target_wp1_y_ + init_pose.pose.position.y;
        target_pose.pose.position.z = target_wp1_z_ + init_pose.pose.position.z;

        while(fabs(target_pose.pose.position.x - current_pose.pose.position.x) > 0.1f 
                || fabs(target_pose.pose.position.y - current_pose.pose.position.y) > 0.1f 
                || fabs(target_pose.pose.position.z - current_pose.pose.position.z) > 0.1f){

            target_pub.publish(target_pose);
            ROS_INFO("Takeoff to fisrt waypoint");
            ros::spinOnce();
            rate.sleep();
        }

        //悬停等待子机离开
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(standby_time_);
        while((ros::Time::now() - start_time) < timeout){
            ROS_INFO("Wait Sub_UAV fly away");
            ros::spinOnce();
            rate.sleep();
        }

        //前往对接点悬停
        while(1){
            target_pose.pose.position.x = target_wp2_x_ + init_pose.pose.position.x;
            target_pose.pose.position.y = target_wp2_y_ + init_pose.pose.position.y;
            target_pose.pose.position.z = target_wp2_z_ + init_pose.pose.position.z;

            target_pub.publish(target_pose);
            ROS_INFO("Wait Docking");
            ros::spinOnce();
            rate.sleep();
        }


    }
}