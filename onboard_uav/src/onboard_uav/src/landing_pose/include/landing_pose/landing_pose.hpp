#pragma once

#include <atomic>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "landing_pose/pos_vel_ekf.hpp"

/**
 * 静态坐标发布：camera_link -- base_link
 */

/**
 * 订阅tf, tag36h11_0 -- camera_link -- base_link
 * 筛选标签位姿，获取body坐标系下的降落目标位姿
 * 对降落目标位姿进行滤波，使用body_vel
 * 将降落目标位姿转换到map坐标系下
 * 发布标签位姿 local ENU
 */

class LandingPose
{
public:
    LandingPose(ros::NodeHandle &nh);
    ~LandingPose();

private:
    void Init();
    struct ekf_param
    {
        int ekf_hz;
        double accel_noise; // 加速度噪声
        double xy_pos_var;  // 位置测量噪声方差系数
        double z_pos_var;   // 位置测量噪声方差系数
        double vel_var;     // 速度测量噪声方差
    };

    struct tag_param
    {
        std::string tag_frame_id;  // 大标签坐标系
        double tag_offset_x;       // 降落目标位姿在大标签坐标系下的x偏置
        double tag_offset_y;       // 降落目标位姿在大标签坐标系下的y偏置
        double tag_offset_z;       // 降落目标位姿在大标签坐标系下的z偏置
        double tag_offset_yaw;     // 降落目标位姿在大标签坐标系下的yaw偏置
        double tag_valid_distance; // 大标签有效检测距离
    };

    ekf_param ekf_param_;
    tag_param tag_param_;

    ros::NodeHandle nh_;
    ros::Subscriber uav_vel_sub_;
    ros::Publisher landing_target_pose_raw_pub_, landing_target_pose_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void UavVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    bool GetTagVel(Eigen::Vector3d &tag_vel);

    void GetTagPose();

    bool TransformLandingTargetPose();

    bool get_new_landing_target_; // 是否获取新的降落目标位姿

    geometry_msgs::TwistStamped uav_vel_;                      // 无人机速度
    geometry_msgs::TransformStamped body_tag_tf_;                   // 坐标变换 base_link --> tag36h11
    geometry_msgs::PoseStamped landing_target_pose_in_tag_frame_;   // 降落目标位姿在大标签坐标系下的位姿
    geometry_msgs::PoseStamped landing_target_body_pose_raw_;  // 降落目标位姿在body坐标系下的原始位姿
    geometry_msgs::PoseStamped landing_target_body_pose_;      // 降落目标位姿在body坐标系下的位姿
    geometry_msgs::PoseStamped landing_target_local_pose_raw_; // 降落目标位姿在local坐标系下的位姿
    geometry_msgs::PoseStamped landing_target_local_pose_;     // 降落目标位姿在local坐标系下的位姿

    std::atomic_flag uav_vel_lock_ = ATOMIC_FLAG_INIT;
    // std::atomic_flag tag_tf_lock_ = ATOMIC_FLAG_INIT; // TODO 在ekf_timer中调用的话不需要锁 降落目标位姿锁

    double uav_vel_timeout_;
    double tf_timeout_;

    PosVelEKF x_ekf_, y_ekf_, z_ekf_;
    double ekf_dt_;        // ekf更新时间间隔
    double ekf_init_time_; // ekf初始化时间
    double ekf_last_time_; // 上一次ekf更新时间
    bool ekf_init_flag_;   // ekf初始化标志位
    ros::Timer ekf_timer_;
    void EkfTimerCallback(const ros::TimerEvent &event);
    bool IsLandingTargetDetected();
};