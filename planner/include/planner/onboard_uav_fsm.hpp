#pragma once

#include <string>
#include <thread>
#include <atomic> //TODO 多线程
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include "quadrotor_msgs/Onboard.h"
#include "quadrotor_msgs/TakeoffLand.h"
#include "quadrotor_msgs/PolyTraj.h"
#include "quadrotor_msgs/FsmState.h"
#include "quadrotor_msgs/GuidanceState.h"
#include "traj_opt/traj_opt.h"
#include "vis_utils/vis_utils.hpp"
#include <nlink_parser/LinktrackNodeframe2.h>


// namespace onboard_uav

class OnboardUavFsm
{
public:
    OnboardUavFsm(ros::NodeHandle &nh);
    ~OnboardUavFsm();

    void Init();

private:
    // 机载无人机状态机
    enum OnboardUavStates
    {
        IDLE = 0, // 空闲状态
        TAKEOFF,  // 起飞，指定位置悬停等待母机信号
        // WAIT_SIGNAL,        // 悬停等待母机信号
        MISSION,   // 执行任务
        REMOTE_GUIDE, // 远程引导，跟随母机飞行轨迹
        SEARCH,
        DOCKING,   // 空中对接
        LAND,      // 就地降落
        FAIL_SAFE, // 故障保护，悬停
    };

    // 对接状态机
    enum DockingStates
    {
        INIT = 0, // 初始化
        RETURN,   // 返航阶段，引导子机飞到母机上方；注意位置偏置，标签可视，防止子机碰撞
        LANDING,  // 降落阶段，进入时初始化为init状态
        COMPLETE, // 降落完成，关闭电机，给母机发送降落完成信号
        RETRY,    // 重试阶段，标签丢失后，重新搜索标签，进入时初始化为init状态
    };

    // 精准降落状态机
    enum class LandingStates
    {
        INIT = 0,
        DESCEND_ABOVE_TARGET, // 在目标上方下降 一段接近
        RE_CURRATE,           // 针对下降过程中xy偏差问题 二段纠正
        FINAL_LANDING         // 最终降落，标签不可见，降落并关闭电机
    };
    // 重试状态机
    enum class RetryStates
    {
        INIT = 0,
        HOVER_SEARCH, // 悬停搜索，不计入重试次数
        CLIMBING      // 爬升，重新进入远程引导阶段，计入重试次数
    };

    OnboardUavStates onboard_uav_state_;
    DockingStates docking_state_;
    LandingStates landing_state_;
    RetryStates retry_state_;

    void RunMissionMode();
    void RunDockingMode();

    void RunDockingIdle();
    void RunDockingReturn();
    void RunDockingLanding();
    void RunDockingComplete();
    void RunDockingRetry();

    void PrintOnboardUavState();
    void PrintUAVPosVel();

    ros::Time retry_start_time_;           // 开始重试的时间
    bool is_docking_retry_;

private:
    std::shared_ptr<vis_utils::VisUtils> vis_ptr_;    // 可视化指针
    std::shared_ptr<traj_opt::TrajOpt> traj_opt_ptr_; // 规划器指针

    Eigen::Vector3d target_pos_, target_vel_; // 目标位置，目标速度
    Eigen::Quaterniond target_q_;             // 目标姿态

    Trajectory poly_traj_; // 多项式轨迹
    Trajectory bvp_traj_;
    int traj_id_;          // 轨迹id

    // 规划轨迹
    bool PlanTrajectory();
    // 发布轨迹
    void PubTrajectory(const ros::Time &plan_start_time);

    // 发布悬停位置
    void PubHoverPos();
    bool hover_flag_; // 悬停标志位

    // 参数
private:
    struct onboard_uav_param
    {

        int uav_id;                  // 母机id
        int target_uav_id;           // 子机编号
        double origin_pos_offset[3]; // 子机原点在母机坐标系下的位置偏置
        double takeoff_height;       // 起飞高度
        double real_takeoff_height;  // 实际起飞高度
    };
    // TODO 速度限制，加速度限制

    struct docking_param
    {
        double return_pos_offset[3];          // 返航位置偏置
        double descend_hor_bias;              // descend水平距离容差
        double descend_ver_bias;              // descend垂直接近高度
        double rec_hor_bias;                  // rec水平距离容差
        double rec_ver_bias;                  // rec垂直接近高度
        double final_hor_bias;                // final水平距离容差
        double final_ver_bias;                // final垂直接近高度
        double allowed_retry_hover_time_s;    // 允许重试悬停时间
        double retry_climb_height;            // 重试爬升高度
        int allowed_retry_num;                // 允许重试次数
        double z_control_dead;                // z轴控制死区
    };

    struct msg_timeout
    {
        double odom;           // 里程计
        double onboard;        // 机载无人机信号
        double landing_target; // 降落目标
    };

    struct remote_guide_param
    {
        double dock_r2m_x;
        double dock_r2m_y;
        double dock_r2m_z;
        int win_anchor_n;
        double res_gate_max;
        double res_gate_rms;
        int mean_cnt;
        double reach_target_thr;
        int settle_cnt;
        double exit_dist_thr;
        double exit_res_thr;
        double step_gamma;
        double step_min;
        double step_max;
        bool search_flag;
        double p_set_1_x;
        double p_set_1_y;
        double p_set_1_z;
        double p_set_2_x;
        double p_set_2_y;
        double p_set_2_z;
        double p_set_3_x;
        double p_set_3_y;
        double p_set_3_z;
        double p_set_4_x;
        double p_set_4_y;
        double p_set_4_z;
        double p_set_5_x;
        double p_set_5_y;
        double p_set_5_z;
        double p_set_6_x;
        double p_set_6_y;
        double p_set_6_z;   
        int go4_point_wait_count;
        bool fly_away_test;
    };

    onboard_uav_param onboard_uav_param_;
    docking_param docking_param_;
    remote_guide_param remote_guide_param_;
    msg_timeout msg_timeout_;

    ros::NodeHandle nh_;
    ros::Subscriber uav_state_sub_, uav_local_pose_sub_, m_uav_local_pose_sub_, uav_local_vel_sub_, uav_odom_sub_, onboard_msg_sub_, landing_target_pose_sub_, uwb_distance_sub_;
    ros::Publisher heartbeat_pub_, takeoff_land_cmd_pub_, trajectory_pub_, onboard_msg_pub_,onboard_uav_state_pub_, mother_move_pub_;
    ros::Publisher remote_ctrl_pub_, fsm_state_pub_;
    ros::ServiceClient arm_disarm_client_;

    ros::Publisher px4_ctl_choose_,position_ctl_pub_;
    std_msgs::Int32 px4_choose_msg;
    mavros_msgs::PositionTarget P_target;; 

    quadrotor_msgs::FsmState fsm_state_;
    quadrotor_msgs::GuidanceState guidance_state_;

    struct RangeSample {
        Eigen::Vector3d p_local;
        double r;
        ros::Time stamp;
    };

    bool rg_first_ref_inited_ = false;
    bool rg_base_inited_      = false;

    Eigen::Vector3d rg_first_ref_world_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rg_base_pos_local_ = Eigen::Vector3d::Zero();
    int rg_stage_ = 0;

    std::deque<RangeSample> rg_anchors_win_;

    bool   rg_have_solution_ = false;
    double rg_last_rms_res_  = std::numeric_limits<double>::infinity();
    double rg_last_max_res_  = std::numeric_limits<double>::infinity();

    bool rg_have_mother_world_ = false;
    Eigen::Vector3d rg_last_mother_world_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d est_dock_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d dock_r2m = Eigen::Vector3d::Zero();

    double uwb_distance = 0.f;
    Eigen::Vector2d circle_search_target = Eigen::Vector2d(0.0, 0.0);   
    bool search_flag = false;

    void Uwb_distance_callback(const std_msgs::Float64 msg); 
    void UavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void UavLocalVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void M_UavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void UavStateCallback(const mavros_msgs::State::ConstPtr &msg);
    void UavOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void OnboardMsgCallback(const quadrotor_msgs::Onboard::ConstPtr &msg);
    void LandingTargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    bool OdomIsReceived(const ros::Time &now_time);
    bool OnboardMsgIsReceived(const ros::Time &now_time);
    bool LandingTargetPoseIsReceived(const ros::Time &now_time);
    bool IsOnboardCommand(const int &flight_command);
    bool IsOnboardStatus(const int &flight_status);
    void Pub_px4_cmd(float target_x, float target_y, float target_z);
    void Pub_FSM_State();
    void Pub_Guidance_State();
    bool Circle_Search();
    void Run_Search();
    void Remote_Guidance();
    bool estimate_mother_from_window(const std::deque<RangeSample>& win, Eigen::Vector3d& mother_local, double& max_residual, double& rms_residual);
    void PubOnboardMsg();
    void PubOnboardUavState();
    void UavDisarm();
    void UpdataFsm(const ros::TimerEvent &event);
    void FillLandingParams();

    mavros_msgs::State uav_state_;                                 // 无人机状态
    geometry_msgs::PoseStamped uav_local_pose_, m_uav_local_pose_; // 无人机本地位置
    geometry_msgs::TwistStamped uav_local_vel_;                    // 无人机本地速度
    nav_msgs::Odometry uav_odom_;                                  // 无人机里程计
    Eigen::Vector3d uav_odom_pos_, m_uav_odom_pos_;                // 无人机里程计位置
    Eigen::Vector3d uav_odom_vel_;                                 // 无人机里程计速度
    Eigen::Vector3d uav_odom_acc_;                                 // TODO 无人机里程计加速度
    Eigen::Quaterniond uav_odom_orient_;                           // 无人机里程计姿态
    quadrotor_msgs::Onboard onboard_received_;                     // 接收到的母机信号
    quadrotor_msgs::Onboard onboard_published_;                    // 发布的母机信号
    geometry_msgs::PoseStamped landing_target_pose_;               // 降落目标位置 local ENU
    bool is_landing_target_pose_updated_;                          // 判断降落目标位姿是否刷新

    bool perform_uav_disarm_;       // 是否上锁
    std::thread uav_disarm_thread_; // 无人机上锁线程
    ros::Timer fsm_timer_; // 定时器
    // 用 fsm_hz_ 控制状态机更新频率
    int fsm_hz_;        // 状态机更新频率
    int replan_hz_;     // 重新规划频率
    bool is_replan_;    // 是否重新规划
    bool is_first_run_; // 是否第一次运行该状态
    ros::Time replan_start_time_;

    traj_opt::LandingParams land_params_;
    int normal_minco_piece_;
    int landing_minco_piece_;
    bool is_landing_ = false;

};
