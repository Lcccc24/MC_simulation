#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class LandingTargetPosePublisher
{
public:
    LandingTargetPosePublisher() : tf_listener_(tf_buffer_)
    {
        // Get parameters from parameter server
        ros::NodeHandle private_nh("~");
        double frequency;
        private_nh.param("timeout", timeout_, 0.2);
        private_nh.param("frequency", frequency, 30.0);

        pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("landing_target_pose_raw", 10);
        timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &LandingTargetPosePublisher::publishPose, this);
    }

    void publishPose(const ros::TimerEvent &)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("map", "tag36h11", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Check if the transform is outdated
        ros::Time now = ros::Time::now();
        if ((now - transform_stamped.header.stamp).toSec() > timeout_)
        {
            ROS_WARN("Transform is outdated");
            return;
        }

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = transform_stamped.header.stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = transform_stamped.transform.translation.x;
        pose_msg.pose.position.y = transform_stamped.transform.translation.y;
        pose_msg.pose.position.z = transform_stamped.transform.translation.z;
        pose_msg.pose.orientation = transform_stamped.transform.rotation;

        pose_publisher_.publish(pose_msg);
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher pose_publisher_;
    ros::Timer timer_;
    double timeout_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_target_pose_publisher");
    LandingTargetPosePublisher landing_target_pose_publisher;
    ros::spin();
    return 0;
}