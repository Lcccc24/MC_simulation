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

    ros::Time docking_landing_start_time_; // 开始降落的时间
    ros::Time final_landing_start_time_;   // 开始最终降落的时间
    ros::Time retry_start_time_;           // 开始重试的时间
    // Eigen::Vector3d retry_pos_;            // 重试时的位置
    // int retry_count_;                      // 重试次数
    bool is_docking_retry_;

private:
    std::shared_ptr<vis_utils::VisUtils> vis_ptr_;    // 可视化指针
    std::shared_ptr<traj_opt::TrajOpt> traj_opt_ptr_; // 规划器指针

    // Eigen::Vector3d uav_pos_, uav_vel_;       // 无人机位置，无人机速度
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
        double allowed_landing_time_s;        // 允许精准降落时间
        double allowed_final_landing_time_s;  // 允许精准降落时间
        double allowed_retry_hover_time_s;    // 允许重试悬停时间
        double retry_climb_height;            // 重试爬升高度
        int allowed_retry_num;                // 允许重试次数
    };

    struct msg_timeout
    {
        double odom;           // 里程计
        double onboard;        // 机载无人机信号
        double landing_target; // 降落目标
    };

    onboard_uav_param onboard_uav_param_;
    docking_param docking_param_;
    msg_timeout msg_timeout_;

    ros::NodeHandle nh_;
    ros::Subscriber uav_state_sub_, uav_local_pose_sub_, uav_local_vel_sub_, uav_odom_sub_, onboard_msg_sub_, landing_target_pose_sub_, uwb_distance_sub_;
    // TODO  在socket通信中，需保证最少发送三次，才能保证消息被接收到
    ros::Publisher heartbeat_pub_, takeoff_land_cmd_pub_, trajectory_pub_, onboard_msg_pub_,onboard_uav_state_pub_, mother_move_pub_;
    ros::Publisher remote_ctrl_pub_, fsm_state_pub_;
    ros::ServiceClient arm_disarm_client_;

    //lc add
    ros::Publisher px4_ctl_choose_,position_ctl_pub_;
    std_msgs::Int32 px4_choose_msg;
    mavros_msgs::PositionTarget P_target;; 

    quadrotor_msgs::FsmState fsm_state_;
    quadrotor_msgs::GuidanceState guidance_state_;

    //void Uwb_distance_callback(const nlink_parser::LinktrackNodeframe2 &msg);
    void Uwb_distance_callback(const std_msgs::Float64 msg); 

    void UavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void UavLocalVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

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
    
    mavros_msgs::State uav_state_;              // 无人机状态
    geometry_msgs::PoseStamped uav_local_pose_; // 无人机本地位置
    geometry_msgs::TwistStamped uav_local_vel_; // 无人机本地速度
    nav_msgs::Odometry uav_odom_;               // 无人机里程计
    Eigen::Vector3d uav_odom_pos_;              // 无人机里程计位置
    Eigen::Vector3d uav_odom_vel_;              // 无人机里程计速度
    Eigen::Vector3d uav_odom_acc_;              // TODO 无人机里程计加速度
    Eigen::Quaterniond uav_odom_orient_;        // 无人机里程计姿态
    quadrotor_msgs::Onboard onboard_received_;  // 接收到的母机信号
    quadrotor_msgs::Onboard onboard_published_; // 发布的母机信号
    // geometry_msgs::TransformStamped landing_target_tf_; // 降落目标tf
    geometry_msgs::PoseStamped landing_target_pose_; // 降落目标位置 local ENU
    bool is_landing_target_pose_updated_;            // 判断降落目标位姿是否刷新

    std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;                // 里程计锁
    std::atomic_flag onboard_msg_lock_ = ATOMIC_FLAG_INIT;         // 机载无人机信号锁
    std::atomic_flag landing_target_pose_lock_ = ATOMIC_FLAG_INIT; // 降落目标位置锁

    void PubOnboardMsg();
    void PubOnboardUavState();

    bool perform_uav_disarm_;       // 是否上锁
    std::thread uav_disarm_thread_; // 无人机上锁线程
    void UavDisarm();

    ros::Timer fsm_timer_; // 定时器
    void UpdataFsm(const ros::TimerEvent &event);
    // 用 fsm_hz_ 控制状态机更新频率
    int fsm_hz_;        // 状态机更新频率
    int replan_hz_;     // 重新规划频率
    bool is_replan_;    // 是否重新规划
    bool is_first_run_; // 是否第一次运行该状态
    ros::Time replan_start_time_;


    //lc add
    void Remote_Guidance();
    //几何法估计
    void geometric_estimate();
    //迭代法估计
    void iterate_estimate();

    // 创建一个包含四个三维向量的数组
    std::array<Eigen::Vector3d, 4> p_set = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, -1.0, 1.0),  
        Eigen::Vector3d(-1.0, -1.0, 0.0),
        Eigen::Vector3d(-1.0, 0.0, 1.0)
 
    };

    std::array<Eigen::Vector3d, 4> pre_vio_p = {
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), 
        Eigen::Vector3d::Zero()
    };
    double uwb_distance = 0.f;
    

    //对接点相对母机的位置
    Eigen::Vector3d dock_r2m = {0.0,0,1.5};
    //几何法子机相对于对接点的估计位置
    Eigen::Vector3d geo_est_c2d = Eigen::Vector3d::Zero();
    Eigen::Vector3d ite_c2d_q = Eigen::Vector3d::Zero();
    Eigen::Vector3d ite_est_p = Eigen::Vector3d::Zero();
    int ite_k = 0;
    //几何法权重
    float geo_p_weight = 0.8;
    //uwb测量四点距离
    double pre_uwb_d[4] = {0,0,0,0};
    float d_stopite = 0.20;
    float d_change = 2.0;

    //远程引导标志位
    int rg_flag = 0;

    Eigen::Vector2d circle_search_target = Eigen::Vector2d(0.0, 0.0);   
    bool search_flag = false;

};
