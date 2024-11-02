#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GazeboModelStates
{
public:
    GazeboModelStates()
    {
        pub = nh.advertise<geometry_msgs::PoseStamped>("/landing_target_pose_true", 10);
        sub1 = nh.subscribe("/gazebo/model_states", 10, &GazeboModelStates::callback1, this);
        sub2 = nh.subscribe("/mavros/local_position/pose", 10, &GazeboModelStates::callback2, this);
    }

    void callback1(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "apriltag")
            {
                apriltag_pose = msg->pose[i];
            }
            else if (msg->name[i] == "iris1")
            {
                iris_pose = msg->pose[i];
            }
        }

        // 计算apriltag相对于iris的位置和姿态
        relative_pose.position.x = apriltag_pose.position.x - iris_pose.position.x;
        relative_pose.position.y = apriltag_pose.position.y - iris_pose.position.y;
        relative_pose.position.z = apriltag_pose.position.z - iris_pose.position.z;

        tf2::Quaternion q1, q2, q_relative;
        tf2::fromMsg(apriltag_pose.orientation, q1);
        tf2::fromMsg(iris_pose.orientation, q2);
        q_relative = q2.inverse() * q1;
        q_relative.normalize();
        relative_pose.orientation = tf2::toMsg(q_relative);
    }

    void callback2(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = msg->pose.position.x + relative_pose.position.x;//+ 0.25;
        pose.pose.position.y = msg->pose.position.y + relative_pose.position.y;
        pose.pose.position.z = msg->pose.position.z + relative_pose.position.z;// + 0.05;

        tf2::Quaternion q1, q2, q_final;
        tf2::fromMsg(msg->pose.orientation, q1);
        tf2::fromMsg(relative_pose.orientation, q2);
        q_final = q1 * q2;
        pose.pose.orientation = tf2::toMsg(q_final);

        pub.publish(pose);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub1, sub2;
    geometry_msgs::Pose apriltag_pose, iris_pose, relative_pose;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_model_states");
    GazeboModelStates gms;
    ros::spin();
    return 0;
}