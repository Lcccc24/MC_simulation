#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/Onboard.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/RCIn.h>
#include <Eigen/Eigen>
#include <vector>

class OnboardPublisher
{
public:
    OnboardPublisher(ros::NodeHandle &nh) : nh_(nh)
    {
        // Initialize parameters from ROS parameter server
        nh_.param("/uav0_onboard/use_param_ctrl", use_param_ctrl_, false); // Use parameters to control the UAV
        nh_.setParam("/uav0_onboard/uav_id", 0);                           // UAV ID
        nh_.setParam("/uav0_onboard/target_uav_id", 1);                    // Target UAV ID
        nh_.setParam("/uav0_onboard/flight_command", 0);                   // Flight modes
        nh_.getParam("/uav0_onboard/uav_id", uav_id_);                     // UAV ID
        nh_.getParam("/uav0_onboard/target_uav_id", target_uav_id_);       // Target UAV ID
        nh_.param("/uav0_onboard/mission_pos_x", mission_pos_x_, 2.0);     // Mission position x
        nh_.param("/uav0_onboard/mission_pos_y", mission_pos_y_, 1.0);     // Mission position y
        nh_.param("/uav0_onboard/mission_pos_z", mission_pos_z_, 2.0);     // Mission position z

        // 订阅无人机的位置话题
        uav_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &OnboardPublisher::uavPoseCallback, this);

        // 订阅无人机0的位置话题
        uav0_pose_sub_ = nh_.subscribe("/uav" + std::to_string(uav_id_) + "/mavros/local_position/pose", 10, &OnboardPublisher::uav0PoseCallback, this);

        // 订阅话题/mavros/rc/in
        px4_rc_in_sub_ = nh_.subscribe("/mavros/rc/in", 10, &OnboardPublisher::px4RCInCallback, this);

        // 订阅/traj_start_trigger
        traj_start_trigger_sub_ = nh_.subscribe("/traj_start_trigger", 10, &OnboardPublisher::trajStartTriggerCallback, this);

        // 订阅话题/uav0/onboard_msg
        onboard_sub_ = nh_.subscribe("/uav" + std::to_string(uav_id_) + "/onboard_msg", 10, &OnboardPublisher::onboardCallback, this);

        // Create a publisher for the onboard message
        onboard_pub_ = nh_.advertise<quadrotor_msgs::Onboard>("/uav" + std::to_string(target_uav_id_) + "/onboard_msg", 10);

        have_msg_ = false;
        pub_onboard_msg_ = false;
        flight_command_ = 0;

        onboard_msg_pub_.uav_id = uav_id_;
        onboard_msg_pub_.target_uav_id = target_uav_id_;

        uav0_pose_.pose.position.x = 0.0;
        uav0_pose_.pose.position.y = 0.0;
        uav0_pose_.pose.position.z = 0.0;
        start_pose_.pose.position.x = 0.0;
        start_pose_.pose.position.y = 0.0;
        start_pose_.pose.position.z = 0.0;

        timer_ = nh_.createTimer(ros::Duration(0.1), &OnboardPublisher::timerCallback, this);

        mission_pos_list_.push_back(Eigen::Vector3d(3.0, 2.0, 2.0));
        mission_pos_list_.push_back(Eigen::Vector3d(5.0, -1.0, 2.0));
        mission_pos_list_.push_back(Eigen::Vector3d(7.0, 3.0, 2.0));
        mission_pos_list_.push_back(Eigen::Vector3d(8.0, 0.0, 2.0));
        mission_pos_list_.push_back(Eigen::Vector3d(8.0, 0.0, 4.0));
        mission_pos_list_.push_back(Eigen::Vector3d(8.5, 1.0, 4.0));
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        nh_.getParam("/uav0_onboard/flight_command", flight_command_); // Flight modes

        // if (flight_command_ == quadrotor_msgs::Onboard::TAKEOFF)
        // {
        //     static bool set_once = false;
        //     if (!set_once)
        //     {
        //         start_pose_ = uav_pose_;
        //         set_once = true;
        //     }
        // }

        if (flight_command_ == quadrotor_msgs::Onboard::TAKEOFF || pub_onboard_msg_)
        {
            runTakeoff();
        }

        if (use_param_ctrl_)
        {
            if (flight_command_ == quadrotor_msgs::Onboard::MISSION)
            {
                runMission();
            }
            else if (flight_command_ == quadrotor_msgs::Onboard::ALLOW_RETURN)
            {
                runDockingReturn();
            }
            else if (flight_command_ == quadrotor_msgs::Onboard::ALLOW_PRECISION_LANDING)
            {
                runDockingLanding();
            }
        }
        else if (onboard_msg_sub_.target_uav_id == uav_id_)
        {
            if (onboard_msg_sub_.flight_status == quadrotor_msgs::Onboard::TAKEOFF_COMPLETE)
            {
                runMission();
            }
            else if (onboard_msg_sub_.flight_status == quadrotor_msgs::Onboard::MISSION_COMPLETE)
            {
                runDockingReturn();
            }
            else if (onboard_msg_sub_.flight_status == quadrotor_msgs::Onboard::LAND_COMPLETE)
            {
            }
            else if (onboard_msg_sub_.flight_status == quadrotor_msgs::Onboard::RETURN_COMPLETE)
            {
                runDockingLanding();
            }
            else if (onboard_msg_sub_.flight_status == quadrotor_msgs::Onboard::PRECISION_LANDING_COMPLETE)
            {
            }
        }
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        uav_pose_ = *msg;
    }

    void uav0PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        uav0_pose_ = *msg;
        have_msg_ = true;
    }

    void px4RCInCallback(const mavros_msgs::RCIn::ConstPtr &msg)
    {
        if (msg->channels[5] > 1600)
        {
            pub_onboard_msg_ = true;
            static bool flag = false;
            if (!flag)
            {
                // start_pose_ = uav_pose_;
                flag = true;
            }
        }
        else
        {
            pub_onboard_msg_ = false;
        }
    }

    void trajStartTriggerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        start_pose_ = *msg;
    }

    void onboardCallback(const quadrotor_msgs::Onboard::ConstPtr &msg)
    {
        onboard_msg_sub_ = *msg;
    }

    void runTakeoff()
    {
        static int takeoff_count = 0;
        if (takeoff_count++ > 10)
        {
            return;
        }
        onboard_msg_pub_.header.stamp = ros::Time::now();
        onboard_msg_pub_.flight_command = quadrotor_msgs::Onboard::TAKEOFF;
        onboard_msg_pub_.flight_status = quadrotor_msgs::Onboard::IDLE;
        onboard_msg_pub_.position.x = start_pose_.pose.position.x;
        onboard_msg_pub_.position.y = start_pose_.pose.position.y;
        onboard_msg_pub_.position.z = start_pose_.pose.position.z + 1.0;
        onboard_pub_.publish(onboard_msg_pub_);
    }

    void runMission()
    {
        // 悬停一会
        static int mission_count = 0;
        if (mission_count++ < 10 && mission_count > 20)
        {
            return;
        }

        onboard_msg_pub_.header.stamp = ros::Time::now();
        onboard_msg_pub_.flight_command = quadrotor_msgs::Onboard::MISSION;
        onboard_msg_pub_.flight_status = quadrotor_msgs::Onboard::TAKEOFF_COMPLETE;
        onboard_msg_pub_.position.x = start_pose_.pose.position.x + mission_pos_x_;
        onboard_msg_pub_.position.y = start_pose_.pose.position.y + mission_pos_y_;
        onboard_msg_pub_.position.z = start_pose_.pose.position.z + mission_pos_z_;
        onboard_pub_.publish(onboard_msg_pub_);
    }
    // void runMission()
    // {
    //     // 悬停一会
    //     // static int mission_count = 0;
    //     // if (mission_count++ < 10)
    //     // {
    //     //     return;
    //     // }

    //     Eigen::Vector3d uav_pose = Eigen::Vector3d(uav_pose_.pose.position.x, uav_pose_.pose.position.y, uav_pose_.pose.position.z);

    //     static int mission_index = 0;
    //     if((uav_pose - mission_pos_list_[mission_index]).norm() < 1.0)
    //     {
    //         if(mission_index < mission_pos_list_.size())
    //         {
    //             mission_index++;
    //         }
    //     }

    //     if(mission_index == mission_pos_list_.size() -2)
    //     {
    //         static int mission_count = 0;
    //         if (mission_count++ < 30)
    //         {
    //             return;
    //         }
    //     }

    //     onboard_msg_pub_.position.x = start_pose_.pose.position.x + mission_pos_list_[mission_index](0);
    //     onboard_msg_pub_.position.y = start_pose_.pose.position.y + mission_pos_list_[mission_index](1);
    //     onboard_msg_pub_.position.z = start_pose_.pose.position.z + mission_pos_list_[mission_index](2);

    //     onboard_msg_pub_.header.stamp = ros::Time::now();
    //     onboard_msg_pub_.flight_command = quadrotor_msgs::Onboard::MISSION;
    //     onboard_msg_pub_.flight_status = quadrotor_msgs::Onboard::TAKEOFF_COMPLETE;
    //     onboard_pub_.publish(onboard_msg_pub_);
    // }

    void runDockingReturn()
    {
        // 先发送 quadrotor_msgs::Onboard::DOCKING
        static int docking_count = 0;
        if (docking_count++ < 30)
        {
            onboard_msg_pub_.flight_command = quadrotor_msgs::Onboard::DOCKING;
        }
        else if (docking_count > 30 && docking_count++ < 40)
        {
            onboard_msg_pub_.flight_command = quadrotor_msgs::Onboard::ALLOW_RETURN;
        }
        else
        {
            return;
        }

        onboard_msg_pub_.header.stamp = ros::Time::now();
        onboard_msg_pub_.flight_status = quadrotor_msgs::Onboard::MISSION_COMPLETE;
        // 返航目标位置，onboard_uav_fsm中会加上子母机起点位置偏移量，从而得到子机坐标系下的返航目标位置
        if (have_msg_)
        {
            onboard_msg_pub_.position.x = uav0_pose_.pose.position.x;
            onboard_msg_pub_.position.y = uav0_pose_.pose.position.y;
            onboard_msg_pub_.position.z = uav0_pose_.pose.position.z;
        }
        else
        {
            onboard_msg_pub_.position.x = start_pose_.pose.position.x;
            onboard_msg_pub_.position.y = start_pose_.pose.position.y;
            onboard_msg_pub_.position.z = start_pose_.pose.position.z;
        }

        onboard_pub_.publish(onboard_msg_pub_);
    }

    void runDockingLanding()
    {
        static int landing_count = 0;
        if (landing_count++ < 10 && landing_count > 20)
        {
            return;
        }
        onboard_msg_pub_.header.stamp = ros::Time::now();
        onboard_msg_pub_.flight_command = quadrotor_msgs::Onboard::ALLOW_PRECISION_LANDING;
        onboard_msg_pub_.flight_status = quadrotor_msgs::Onboard::RETURN_COMPLETE;
        onboard_msg_pub_.position.x = start_pose_.pose.position.x + uav0_pose_.pose.position.x;
        onboard_msg_pub_.position.y = start_pose_.pose.position.y + uav0_pose_.pose.position.y;
        onboard_msg_pub_.position.z = start_pose_.pose.position.z + uav0_pose_.pose.position.z;
        onboard_pub_.publish(onboard_msg_pub_);
    }

private:
    ros::NodeHandle nh_;

    ros::Timer timer_;

    ros::Subscriber uav_pose_sub_;
    ros::Subscriber uav0_pose_sub_;
    ros::Subscriber px4_rc_in_sub_;
    ros::Subscriber traj_start_trigger_sub_;
    ros::Subscriber onboard_sub_;
    ros::Publisher onboard_pub_;

    geometry_msgs::PoseStamped uav_pose_;
    geometry_msgs::PoseStamped uav0_pose_;
    geometry_msgs::PoseStamped start_pose_;
    quadrotor_msgs::Onboard onboard_msg_sub_;
    quadrotor_msgs::Onboard onboard_msg_pub_;

    bool pub_onboard_msg_;

    bool use_param_ctrl_;
    bool have_msg_;
    int uav_id_;
    int target_uav_id_;
    int flight_command_;
    double mission_pos_x_;
    double mission_pos_y_;
    double mission_pos_z_;
    std::vector<Eigen::Vector3d> mission_pos_list_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "onboard_publisher");
    ros::NodeHandle nh;

    OnboardPublisher publisher(nh);
    ros::spin();
    return 0;
}
