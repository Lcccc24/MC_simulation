#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

namespace uav_utils
{
    class PoseEKFTf : public nodelet::Nodelet
    {

    public:
        PoseEKFTf() {}
        ~PoseEKFTf() {}

        virtual void onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();

            pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/landing_target_pose", 10);
            odom_sub_ = nh.subscribe("/odom", 1, &PoseEKFTf::odomCallback, this);
        }

        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = odom_msg->header;
            pose_msg.pose = odom_msg->pose.pose;

            try
            {
                tf_listener_.transformPose("map", pose_msg, pose_msg);
                pose_pub_.publish(pose_msg);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

    private:
        ros::Subscriber odom_sub_;
        ros::Publisher pose_pub_;
        tf::TransformListener tf_listener_;
    };
} // namespace uav_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_utils::PoseEKFTf, nodelet::Nodelet)