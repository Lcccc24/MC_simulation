#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include "vision_pose/eskf_jain.hpp"
#include "sophus/se3.hpp"

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
        // bool eskf_init;                 // 重置ESKF
        int eskf_hz;
        bool use_imu0;                  // 是否使用uav0的IMU数据
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
    ros::Subscriber uav_imu_sub_, uav0_imu_sub_, uav_local_pos_sub_ , tag_detection_sub_ , gazebo_true_sub_;
    ros::Publisher landing_target_pose_raw_pub_, landing_target_pose_pub_;
    geometry_msgs::TransformStamped camera_to_tag_tf_;
    //tf2_ros::Buffer tf_buffer_;
    //tf2_ros::TransformListener tf_listener_;

    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void Imu0Callback(const sensor_msgs::Imu::ConstPtr &msg);
    void LocalPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void TagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void GazeboTrueCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

    bool GetTagPose();

    bool IsLandingTargetDetected();
    bool IsTagPoseValid();

    void CalculateLandingTargetPose();
    void CalculateLandingTargetPoseRaw();

    void UpdateParam();
    void SetEskfParam();

    sensor_msgs::Imu imu_;
    geometry_msgs::PoseStamped tag_pose_, last_tag_pose_;
    geometry_msgs::PoseStamped uav_local_pos_;

    geometry_msgs::PoseStamped landing_target_pose_raw_, landing_target_pose_;

    Eigen::Matrix3d R_sc_, R_em_, R_ml_, R_mt_;
    Eigen::Quaterniond q_em_;

    Sophus::SE3d nominal_state_;
    Sophus::SE3d p_sm_R_ct_;

    double imu_timeout_;
    double tf_timeout_;

    bool imu_init_flag_;
    bool imu0_init_flag_;
    // bool tf_init_flag_;
    bool get_new_landing_target_;

    ros::Time get_new_landing_target_time_;

    double eskf_dt_;
    double eskf_init_time_;
    bool eskf_init_flag_;

    ESKFD eskf_;
    ros::Timer eskf_timer_;
    void EskfTimerCallback(const ros::TimerEvent &event);
};
