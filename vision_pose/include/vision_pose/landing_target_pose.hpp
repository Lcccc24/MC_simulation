#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include "vision_pose/eskf.hpp"
#include "sophus/se3.hpp"
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <random>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <gazebo_msgs/ModelStates.h>

class LandingTargetPose
{
public:
    LandingTargetPose(ros::NodeHandle &nh);
    ~LandingTargetPose();

private:
    void getParam();
    void Init();
    struct eskf_param
    {
        int eskf_hz;
        double gyro_var;                // 陀螺测量方差
        double acce_var;                // 加计测量方差
        double bias_gyro_var;           // 陀螺零偏游走方差
        double bias_acce_var;           // 加计零偏游走方差
        Eigen::Vector3d gyro_bias;      // 陀螺零偏
        Eigen::Vector3d acce_bias;      // 加计零偏
        bool update_bias_gyro;          // 是否更新陀螺零偏
        bool update_bias_acce;          // 是否更新加计零偏
        double vision_xy_noise;         // 视觉测量xy轴位置噪声
        double vision_z_noise;          // 视觉测量z轴位置噪声
        double vision_roll_pitch_noise; // 视觉测量横滚俯仰噪声
        double vision_yaw_noise;        // 视觉测量偏航噪声
        double uwb_noise;                 // UWB噪声
        double relative_position_xy_noise; // 相对位置xy观测噪声
        double relative_position_z_noise; // 相对位置z观测噪声
        double gravity;                 // 重力加速度
    };

    struct tag_param
    {
        std::string topic_name;  // 标签检测话题
        std::string tag_frame_id;  // 标签坐标系
        double tag_to_landing_x;   // 降落目标位姿在标签坐标系下的x偏置
        double tag_to_landing_y;   // 降落目标位姿在标签坐标系下的y偏置
        double tag_to_landing_z;   // 降落目标位姿在标签坐标系下的z偏置
        double tag_to_landing_yaw; // 降落目标位姿在标签坐标系下的yaw偏置
        double tag_valid_distance; // 标签有效检测距离
    };

    struct frame_param
    {
        std::string camera_frame_id; // 相机坐标系
        Eigen::Vector3d body_to_camera_p;
        Eigen::Quaterniond body_to_camera_q;
        Eigen::Vector3d uav0_to_tag_p;
        Eigen::Quaterniond uav0_to_tag_q;
        Eigen::Vector3d uav0_to_landing_p;
        Eigen::Quaterniond uav0_to_landing_q;
    };

    eskf_param eskf_param_;
    tag_param tag_param_;
    frame_param frame_param_;

    ros::NodeHandle nh_;
    ros::Subscriber uav_imu_sub_, uav_local_pos_sub_, m_uav_local_pos_sub_, tag_detection_sub_, uwb_sub_;
    ros::Publisher landing_target_pose_raw_pub_, landing_target_pose_pub_, landing_relative_odom_pub_;

    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void LocalPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void M_LocalPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void TagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void UwbDistanceCallback(const std_msgs::Float64::ConstPtr &msg);

    bool IsTagPoseValid();
    void UpdateRelativePosition();

    void CalculateLandingTargetPose();
    void CalculateLandingTargetPoseRaw();

    void UpdateParam();
    void SetEskfParam();

    sensor_msgs::Imu imu_;
    geometry_msgs::PoseStamped tag_pose_, last_tag_pose_;
    geometry_msgs::PoseStamped uav_local_pos_, m_uav_local_pos_;
    Eigen::Vector3d relative_pos_;

    geometry_msgs::PoseStamped landing_target_pose_raw_, landing_target_pose_, landing_target_relative_odom_;

    Eigen::Matrix3d R_sc_, R_em_, R_ml_, R_mt_;
    Eigen::Quaterniond q_em_;

    Sophus::SE3d nominal_state_;
    Sophus::SE3d p_sm_R_ct_;

    bool imu_init_flag_;

    double eskf_dt_;
    double eskf_init_time_;
    bool eskf_init_flag_;
    bool eskf_enabled_ = false; 

    ESKFD eskf_;
    ros::Timer eskf_timer_;
    void EskfTimerCallback(const ros::TimerEvent &event);

    //lc add: 视觉监测相关变量
    double last_vision_time_ = 0.0;    
    double vision_timeout_ = 1.0;      
    bool vision_valid_ = false;         
    bool get_new_landing_target_;

    // uwb观测相关
    double uwb_distance_ = 0.0;
    double uwb_timeout_ = 1.0;
    double last_uwb_time_ = 0.0;
    bool uwb_valid_ = false;
    bool get_new_uwb_ = false;

    // 相对位置观测相关
    bool get_new_relative_position_ = false;
    double last_relative_position_time_ = 0.0;
    double relative_position_timeout_ = 1.0;
    bool relative_position_valid_ = false;
};
