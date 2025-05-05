#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GazeboModelStates
{
public:
    GazeboModelStates()
    {
        relative_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/relative_pose/gazebo_true", 10);
        
        Apritag_odom_pub = nh.advertise<nav_msgs::Odometry>("/Apritag_odom/gazebo_true", 10);
        AVC_odom_pub = nh.advertise<nav_msgs::Odometry>("/AVC_odom/gazebo_true", 10);
        Sub_UAV_odom_pub = nh.advertise<nav_msgs::Odometry>("Sub_UAV_odom/gazebo_true", 10);

        gazebo_true_sub = nh.subscribe("/gazebo/model_states", 10, &GazeboModelStates::StatesCallback, this);
    }

    void StatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {

        //ROS_INFO("true");

        //iris0 : AVC
        //iris1 : Sub-UAV
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "apriltag")
            {
                Apriltag_pose = msg->pose[i];
                Apriltag_twist = msg->twist[i];
            }
            else if (msg->name[i] == "iris0")
            {
                AVC_pose = msg->pose[i];
                AVC_twist = msg->twist[i];
            }
            else if (msg->name[i] == "iris1")
            {
                Sub_UAV_pose = msg->pose[i];
                Sub_UAV_twist = msg->twist[i];
            }
        }

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose = Apriltag_pose;
        //to world frame 
        odom_msg.pose.pose.position.x -= 1.0f;
        odom_msg.twist.twist = Apriltag_twist;
        Apritag_odom_pub.publish(odom_msg);

        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose = AVC_pose;
        odom_msg.twist.twist = AVC_twist;
        AVC_odom_pub.publish(odom_msg);

        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose = Sub_UAV_pose;
        odom_msg.twist.twist = Sub_UAV_twist;
        Sub_UAV_odom_pub.publish(odom_msg);

        // 计算apriltag相对于Sub_UAV的位置和姿态
        relative_pose.position.x = Apriltag_pose.position.x - Sub_UAV_pose.position.x;
        relative_pose.position.y = Apriltag_pose.position.y - Sub_UAV_pose.position.y;
        relative_pose.position.z = Apriltag_pose.position.z - Sub_UAV_pose.position.z;

        tf2::Quaternion q1, q2, q_relative;
        tf2::fromMsg(Apriltag_pose.orientation, q1);
        tf2::fromMsg(Sub_UAV_pose.orientation, q2);
        q_relative = q2.inverse() * q1;
        q_relative.normalize();
        relative_pose.orientation = tf2::toMsg(q_relative);

        geometry_msgs::PoseStamped relative_pose_msg;
        relative_pose_msg.header.stamp = ros::Time::now(); 
        relative_pose_msg.pose = relative_pose;
        relative_pose_pub.publish(relative_pose_msg);
    }



private:
    ros::NodeHandle nh;
    ros::Publisher relative_pose_pub, Apritag_odom_pub, AVC_odom_pub, Sub_UAV_odom_pub;


    ros::Subscriber gazebo_true_sub;
    geometry_msgs::Pose Apriltag_pose, AVC_pose, Sub_UAV_pose, relative_pose;
    geometry_msgs::Twist Apriltag_twist, AVC_twist, Sub_UAV_twist;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_model_states");
    GazeboModelStates gms;
    ros::spin();
    return 0;
}