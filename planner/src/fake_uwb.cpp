#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 dis_msg;
ros::Publisher fakeuwb_dis_pub;
ros::Subscriber uav0_p_sub_, uav1_p_sub_;
Eigen::Vector3d uav0_pos_ = Eigen::Vector3d::Zero();
Eigen::Vector3d uav1_pos_ = Eigen::Vector3d::Zero();
double fake_uwb_d = 0.f;


void Uav0LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav0_pos_.x() = msg->pose.position.x + 1; 
    uav0_pos_.y() = msg->pose.position.y; 
    uav0_pos_.z() = msg->pose.position.z; 
}

void Uav1LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav1_pos_.x() = msg->pose.position.x;
    uav1_pos_.y() = msg->pose.position.y; 
    uav1_pos_.z() = msg->pose.position.z; 
}

void cal_fakeuwb_distance()
{
    fake_uwb_d = (uav0_pos_ - uav1_pos_).norm();
    dis_msg.data = fake_uwb_d;
    fakeuwb_dis_pub.publish(dis_msg);
    ROS_INFO("fakeuwbdistance:%f",fake_uwb_d);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_uwb_node");
    ros::NodeHandle nh;
    uav0_p_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/pose", 1, &Uav0LocalPoseCallback);
    uav1_p_sub_ = nh.subscribe("/AVC/mavros/local_position/pose", 1, &Uav1LocalPoseCallback);
    fakeuwb_dis_pub = nh.advertise<std_msgs::Float64>("/fake_uwb_distance",10);
    ros::Rate rate(50);

    while(1)
    {
        cal_fakeuwb_distance();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}