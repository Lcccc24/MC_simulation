#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>

class PoseControlNode
{
public:
    PoseControlNode() : nh_("~")
    {

        nh_.param<std::string>("target_uav_state_topic", state_topic_, "/uav0/mavros/state");
        // nh_.param<std::string>("target_uav_pose_topic", pose_topic_, "/uav0/mavros/local_position/pose");
        nh_.param<std::string>("target_uav_name", uav_name_, "iris0");
        nh_.param<std::string>("target_model_name", model_name_, "apriltag");

        state_sub_ = nh_.subscribe<mavros_msgs::State>(state_topic_, 10, &PoseControlNode::stateCallback, this);
        // pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(pose_topic_, 10, &PoseControlNode::poseCallback, this);
        // 订阅gazebo真值
        pose_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &PoseControlNode::poseCallback, this);
        set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        set_model_state_.request.model_state.model_name = model_name_;
    }

private:
    void stateCallback(const mavros_msgs::State::ConstPtr &state_msg)
    {
        current_state_ = *state_msg;
    }

    // void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    // {
    //     //

    //     current_pose_ = *pose_msg;

    //     if (!current_state_.armed)
    //     {
    //         return;
    //     }

    //     set_model_state_.request.model_state.pose = current_pose_.pose;
    //     set_model_state_.request.model_state.pose.position.z += 0.3;

    //     // if (!set_model_state_client_.waitForExistence(ros::Duration(1.0)))
    //     // {
    //     //     ROS_ERROR("Failed to connect to Set Model State Service");
    //     //     return;
    //     // }

    //     if (!set_model_state_client_.call(set_model_state_))
    //     {
    //         ROS_ERROR("Failed to call Set Model State Service");
    //     }
    // }

    void poseCallback(const gazebo_msgs::ModelStates::ConstPtr &pose_msg)
    {
        // if (!current_state_.armed)
        // {
        //     return;
        // }

        // 查找模型为uav0的索引
        for (int i = 0; i < pose_msg->name.size(); i++)
        {
            if (pose_msg->name[i] == uav_name_)
            {
                current_pose_.pose = pose_msg->pose[i];
                set_model_state_.request.model_state.pose = current_pose_.pose;

                // set_model_state_.request.model_state.pose.position = current_pose_.pose.position;
                // set_model_state_.request.model_state.pose.orientation.x = 0;
                // set_model_state_.request.model_state.pose.orientation.y = 0;
                // set_model_state_.request.model_state.pose.orientation.z = 0;
                // set_model_state_.request.model_state.pose.orientation.w = 1;

                set_model_state_.request.model_state.pose.position.z += 0.10;

                if (!set_model_state_client_.call(set_model_state_))
                {
                    ROS_ERROR("Failed to call Set Model State Service");
                }

                // {
                //     gazebo_msgs::SetModelState set_model_state_temp;
                //     std::string temp_name;

                //     temp_name = "box1";
                //     set_model_state_temp.request.model_state.model_name = temp_name;
                //     set_model_state_temp.request.model_state.pose = set_model_state_.request.model_state.pose;
                //     set_model_state_temp.request.model_state.pose.position.z += 0.05;
                //     set_model_state_temp.request.model_state.pose.position.x += 0.28;
                //     if (!set_model_state_client_.call(set_model_state_temp))
                //     {
                //         ROS_ERROR("Failed to call Set Model State Service");
                //     }
                // }
                break;
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::ServiceClient set_model_state_client_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    gazebo_msgs::SetModelState set_model_state_;
    std::string state_topic_;
    std::string uav_name_;
    std::string model_name_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_pose_ctrl");
    PoseControlNode node;
    ros::spin();
    return 0;
}