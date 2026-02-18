#include "planner/onboard_uav_fsm.hpp"

/**
 * @brief 构造函数
 * @param nh ROS 节点句柄
 */
OnboardUavFsm::OnboardUavFsm(ros::NodeHandle &nh)
{
    nh_ = nh; // TODO: check if this is necessary
    nh.param("/uav_id", onboard_uav_param_.uav_id, 1);
    nh.param("/target_uav_id", onboard_uav_param_.target_uav_id, 0);
    nh.param("uav/origin_pos_offset_x", onboard_uav_param_.origin_pos_offset[0], 1.0);
    nh.param("uav/origin_pos_offset_y", onboard_uav_param_.origin_pos_offset[1], 0.0);
    nh.param("uav/origin_pos_offset_z", onboard_uav_param_.origin_pos_offset[2], 0.0);
    nh.param("uav/takeoff_height", onboard_uav_param_.takeoff_height, 1.0);

    nh.param("docking/return_pos_offset_x", docking_param_.return_pos_offset[0], 0.0);
    nh.param("docking/return_pos_offset_y", docking_param_.return_pos_offset[1], 0.0);
    nh.param("docking/return_pos_offset_z", docking_param_.return_pos_offset[2], 1.0);
    nh.param("docking/descend_hor_bias", docking_param_.descend_hor_bias, 0.4);
    nh.param("docking/descend_ver_bias", docking_param_.descend_ver_bias, 0.7);
    nh.param("docking/rec_hor_bias", docking_param_.rec_hor_bias, 0.3);
    nh.param("docking/rec_ver_bias", docking_param_.rec_ver_bias, 0.35);
    nh.param("docking/final_hor_bias", docking_param_.final_hor_bias, 0.14);
    nh.param("docking/final_ver_bias", docking_param_.final_ver_bias, 0.14);
    nh.param("docking/z_control_dead", docking_param_.z_control_dead, 0.025);
    nh.param("docking/allowed_retry_hover_time_s", docking_param_.allowed_retry_hover_time_s, 10.0);
    nh.param("docking/retry_climb_height", docking_param_.retry_climb_height, 2.0);
    nh.param("docking/allowed_retry_num", docking_param_.allowed_retry_num, 10);

    nh.param("remote_guide/dock_r2m_x", remote_guide_param_.dock_r2m_x, 0.0);
    nh.param("remote_guide/dock_r2m_y", remote_guide_param_.dock_r2m_y, 0.0);
    nh.param("remote_guide/dock_r2m_z", remote_guide_param_.dock_r2m_z, 2.0);
    nh.param("remote_guide/win_anchor_n", remote_guide_param_.win_anchor_n, 4);
    nh.param("remote_guide/search_flag", remote_guide_param_.search_flag, true);
    nh.param("remote_guide/mean_cnt", remote_guide_param_.mean_cnt, 30);
    nh.param("remote_guide/reach_target_thr", remote_guide_param_.reach_target_thr, 0.2);
    nh.param("remote_guide/settle_cnt", remote_guide_param_.settle_cnt, 20);
    nh.param("remote_guide/res_gate_max", remote_guide_param_.res_gate_max, 0.5);
    nh.param("remote_guide/res_gate_rms", remote_guide_param_.res_gate_rms, 0.6);
    nh.param("remote_guide/exit_dist_thr", remote_guide_param_.exit_dist_thr, 0.5);
    nh.param("remote_guide/exit_res_thr", remote_guide_param_.exit_res_thr, 0.05);
    nh.param("remote_guide/step_gamma", remote_guide_param_.step_gamma, 0.05);
    nh.param("remote_guide/step_min", remote_guide_param_.step_min, 0.5);
    nh.param("remote_guide/step_max", remote_guide_param_.step_max, 2.0);
    nh.param("remote_guide/p_set_1_x", remote_guide_param_.p_set_1_x, 0.5);
    nh.param("remote_guide/p_set_1_y", remote_guide_param_.p_set_1_y, 0.0);
    nh.param("remote_guide/p_set_1_z", remote_guide_param_.p_set_1_z, 0.3);
    nh.param("remote_guide/p_set_2_x", remote_guide_param_.p_set_2_x, 0.25);
    nh.param("remote_guide/p_set_2_y", remote_guide_param_.p_set_2_y, 0.43);
    nh.param("remote_guide/p_set_2_z", remote_guide_param_.p_set_2_z, 0.3);
    nh.param("remote_guide/p_set_3_x", remote_guide_param_.p_set_3_x, -0.25);
    nh.param("remote_guide/p_set_3_y", remote_guide_param_.p_set_3_y, 0.43);
    nh.param("remote_guide/p_set_3_z", remote_guide_param_.p_set_3_z, 0.0);
    nh.param("remote_guide/p_set_4_x", remote_guide_param_.p_set_4_x, -0.5);
    nh.param("remote_guide/p_set_4_y", remote_guide_param_.p_set_4_y, 0.0);
    nh.param("remote_guide/p_set_4_z", remote_guide_param_.p_set_4_z, 0.0);
    nh.param("remote_guide/p_set_5_x", remote_guide_param_.p_set_5_x, -0.25);
    nh.param("remote_guide/p_set_5_y", remote_guide_param_.p_set_5_y, -0.43);
    nh.param("remote_guide/p_set_5_z", remote_guide_param_.p_set_5_z, -0.3);
    nh.param("remote_guide/p_set_6_x", remote_guide_param_.p_set_6_x, 0.25);
    nh.param("remote_guide/p_set_6_y", remote_guide_param_.p_set_6_y, -0.43);
    nh.param("remote_guide/p_set_6_z", remote_guide_param_.p_set_6_z, -0.3);
    nh.param("remote_guide/p_set_7_x", remote_guide_param_.p_set_7_x, -0.25);
    nh.param("remote_guide/p_set_7_y", remote_guide_param_.p_set_7_y, -0.43);
    nh.param("remote_guide/p_set_7_z", remote_guide_param_.p_set_7_z, -0.3);
    nh.param("remote_guide/p_set_8_x", remote_guide_param_.p_set_8_x, 0.25);
    nh.param("remote_guide/p_set_8_y", remote_guide_param_.p_set_8_y, -0.43);
    nh.param("remote_guide/p_set_8_z", remote_guide_param_.p_set_8_z, -0.3);
    nh.param("remote_guide/go4_point_wait_count", remote_guide_param_.go4_point_wait_count, 60);
    nh.param("remote_guide/fly_away_test", remote_guide_param_.fly_away_test, false);

    nh.param("mission/mission_pt1_x", mission_param_.mission_pt1_x, 3.0);
    nh.param("mission/mission_pt1_y", mission_param_.mission_pt1_y, 0.0);
    nh.param("mission/mission_pt1_z", mission_param_.mission_pt1_z, 2.0);
    nh.param("mission/mission_pt2_x", mission_param_.mission_pt2_x, 0.0);
    nh.param("mission/mission_pt2_y", mission_param_.mission_pt2_y, 10.0);
    nh.param("mission/mission_pt2_z", mission_param_.mission_pt2_z, 2.0);
    nh.param("mission/mission_pt3_x", mission_param_.mission_pt3_x, 3.0);
    nh.param("mission/mission_pt3_y", mission_param_.mission_pt3_y, 10.0);
    nh.param("mission/mission_pt3_z", mission_param_.mission_pt3_z, 2.0);

    nh.param("trajectory/normal_minco_piece", normal_minco_piece_, 10);
    nh.param("trajectory/landing_minco_piece", landing_minco_piece_, 5);

    nh.param("msg_timeout/odom", msg_timeout_.odom, 0.5);
    nh.param("msg_timeout/onboard", msg_timeout_.onboard, 0.5);
    nh.param("msg_timeout/landing_target", msg_timeout_.landing_target, 0.1);

    uav_local_pose_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/pose", 1, &OnboardUavFsm::UavLocalPoseCallback, this);
    uav_local_vel_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/velocity_local", 1, &OnboardUavFsm::UavLocalVelCallback, this);
    m_uav_local_pose_sub_ = nh.subscribe("/AVC/mavros/local_position/pose", 1, &OnboardUavFsm::M_UavLocalPoseCallback, this);
    uav_state_sub_ = nh.subscribe("/Sub_UAV/mavros/state", 1, &OnboardUavFsm::UavStateCallback, this);
    uav_odom_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/odom", 1, &OnboardUavFsm::UavOdomCallback, this);
    std::string onboard_msg_sub_name = "/uav" + std::to_string(onboard_uav_param_.uav_id) + "/onboard_msg";
    onboard_msg_sub_ = nh.subscribe(onboard_msg_sub_name, 1, &OnboardUavFsm::OnboardMsgCallback, this);
    // std::string landing_target_pose_topic_name = std::to_string(onboard_uav_param_.uav_id) + "/landing_target_pose";
    landing_target_eskf_sub_ = nh.subscribe("/landing_target_pose/ESKF", 1, &OnboardUavFsm::LandingTargetEskfCallback, this);
    landing_target_vision_sub_ = nh.subscribe("/landing_target_pose_raw", 1, &OnboardUavFsm::LandingTargetVisionsCallback, this);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/heartbeat", 1); 
    takeoff_land_cmd_pub_ = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    trajectory_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("/trajectory", 1);
    bvp_traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("/bvp_traj", 1);
    std::string onboard_msg_pub_name = "/uav" + std::to_string(onboard_uav_param_.target_uav_id) + "/onboard_msg";
    onboard_msg_pub_ = nh.advertise<quadrotor_msgs::Onboard>(onboard_msg_pub_name, 1);
    arm_disarm_client_ = nh.serviceClient<mavros_msgs::CommandLong>("/Sub_UAV/mavros/cmd/command");
    onboard_uav_state_pub_ = nh.advertise<std_msgs::Float32>("/onboard_uav_state", 1);
    px4_ctl_choose_ = nh.advertise<std_msgs::Int32>("/px4_ctl_choose", 10);
    position_ctl_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/Sub_UAV/mavros/setpoint_raw/local", 10);
    mother_move_pub_ = nh.advertise<std_msgs::Int32>("/mother_move/cmd",1);
    eskf_actitve_pub_ = nh.advertise<std_msgs::Bool>("/eskf_active", 1);
    //uwb_distance_sub_  = nh.subscribe("nlink_linktrack_nodeframe2", 1000, &OnboardUavFsm::Uwb_distance_callback,this);
    uwb_distance_sub_  = nh.subscribe("/fake_uwb_distance", 10, &OnboardUavFsm::Uwb_distance_callback,this);
    //远程引导标志位pub与进入降落标志位pub 
    remote_ctrl_pub_ = nh.advertise<quadrotor_msgs::GuidanceState>("/remote_ctrl/state", 1);
    fsm_state_pub_ = nh.advertise<quadrotor_msgs::FsmState>("/fsm_state", 1);
    coord_align_pub_ = nh.advertise<geometry_msgs::Vector3>("/coord_align", 1);
    mother_ready_sub_ = nh.subscribe("/mother_arrived", 1, &OnboardUavFsm::MotherReadyCallback, this);
    
    // 创建路径规划器实例
    traj_opt_ptr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    // 创建可视化实例
    vis_ptr_ = std::make_shared<vis_utils::VisUtils>(nh);
    // 初始化线程
    uav_disarm_thread_ = std::thread(&OnboardUavFsm::UavDisarm, this);

    fsm_hz_ = 20;
    replan_hz_ = 10;
    fsm_timer_ = nh.createTimer(ros::Duration(1.0 / fsm_hz_), &OnboardUavFsm::UpdataFsm, this);

    Init();
}

/**
 * @brief 析构函数
 */
OnboardUavFsm::~OnboardUavFsm()
{
    uav_disarm_thread_.join();
}

/**
 * @brief 初始化函数
 */
void OnboardUavFsm::Init()
{
    onboard_uav_state_ = OnboardUavStates::IDLE;
    docking_state_ = DockingStates::INIT;
    landing_state_ = LandingStates::INIT;
    retry_state_ = RetryStates::INIT;

    traj_id_ = 0;
    is_landing_target_eskf_updated_ = false;
    is_landing_target_vision_updated_ = false;
    hover_flag_ = false;
    is_docking_retry_ = false;
    uav_local_pose_ = {};
    uav_local_vel_ = {};
    uav_odom_ = {};
    onboard_received_ = {};
    landing_target_eskf_ = {};
    landing_target_vision_ = {};
    target_pos_ = Eigen::Vector3d::Zero();
    target_vel_ = Eigen::Vector3d::Zero();
    target_q_ = Eigen::Quaterniond::Identity();

    perform_uav_disarm_ = false;
    is_replan_ = true;
    is_first_run_ = true;
    replan_start_time_ = ros::Time(0.0);

    dock_r2m.x() = remote_guide_param_.dock_r2m_x;
    dock_r2m.y() = remote_guide_param_.dock_r2m_y;
    dock_r2m.z() = remote_guide_param_.dock_r2m_z;

    mission_pt[0].x() = mission_param_.mission_pt1_x;
    mission_pt[0].y() = mission_param_.mission_pt1_y;
    mission_pt[0].z() = mission_param_.mission_pt1_z;
    mission_pt[1].x() = mission_param_.mission_pt2_x;
    mission_pt[1].y() = mission_param_.mission_pt2_y;
    mission_pt[1].z() = mission_param_.mission_pt2_z;
    mission_pt[2].x() = mission_param_.mission_pt3_x;
    mission_pt[2].y() = mission_param_.mission_pt3_y;
    mission_pt[2].z() = mission_param_.mission_pt3_z;

    px4_choose_msg.data = 0;
    px4_ctl_choose_.publish(px4_choose_msg);
}

// 四元数 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
static Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q)
{
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void OnboardUavFsm::Uwb_distance_callback(const std_msgs::Float64 msg) 
{ 
    // if (!msg.nodes.empty()) 
    // {
    //     double distance = msg.nodes[0].dis;
    //     if (std::isfinite(distance)) 
    //     {
    //         uwb_distance = distance;
    //         //ROS_INFO("Received distance: %f", uwb_distance);
    //     } 
    //     else 
    //     ROS_WARN("Received invalid distance: %f", distance);
    // }
    // else 
    //     ROS_WARN("Received empty UWB nodes array");

    uwb_distance = msg.data;
    //ROS_INFO("uwbdistance %f",uwb_distance);
}
    
/**
 * @brief 无人机状态订阅回调函数
 * @param msg 无人机状态消息
 */
void OnboardUavFsm::UavStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    uav_state_ = *msg;
}

void OnboardUavFsm::UavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_local_pose_ = *msg;
    uav_odom_pos_ << uav_local_pose_.pose.position.x, uav_local_pose_.pose.position.y, uav_local_pose_.pose.position.z;
}
void OnboardUavFsm::UavLocalVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_local_vel_ = *msg;
}

void OnboardUavFsm::M_UavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_uav_local_pose_ = *msg;
    m_uav_odom_pos_ << m_uav_local_pose_.pose.position.x, m_uav_local_pose_.pose.position.y, m_uav_local_pose_.pose.position.z;
}

/**
 * @brief 无人机里程计订阅回调函数
 * @param msg 无人机里程计消息
 */
void OnboardUavFsm::UavOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_odom_ = *msg;
}

/**
 * @brief 母机指令订阅回调函数
 * @param msg 母机指令消息
 */
void OnboardUavFsm::OnboardMsgCallback(const quadrotor_msgs::Onboard::ConstPtr &msg)
{
    // TODO 筛选无人机 ID
    onboard_received_ = *msg;
    //ROS_INFO("onboard_received_ : %f,%f,%f",onboard_received_.position.x,onboard_received_.position.y,onboard_received_.position.z);
}

void OnboardUavFsm::MotherReadyCallback(const std_msgs::Bool::ConstPtr &msg)
{
    mother_ready = msg->data;
}

/**
 * @brief 降落目标位置订阅回调函数
 * @param msg 降落目标位置消息
 */
void OnboardUavFsm::LandingTargetEskfCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    landing_target_eskf_ = *msg;
    is_landing_target_eskf_updated_ = true;
}

void OnboardUavFsm::LandingTargetVisionsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    landing_target_vision_ = *msg;
    is_landing_target_vision_updated_ = true;
}

/**
 * @brief 判断里程计是否接收到
 * @param now_time 时间
 * @return bool 是否接收到
 */
bool OnboardUavFsm::OdomIsReceived(const ros::Time &now_time)
{
    bool is_received = now_time.toSec() - uav_local_pose_.header.stamp.toSec() < msg_timeout_.odom;
    return is_received;
}

/**
 * @brief 判断母机指令是否接收到
 * @param now_time 时间
 * @return bool 是否接收到
 */
bool OnboardUavFsm::OnboardMsgIsReceived(const ros::Time &now_time)
{
    bool is_received = now_time.toSec() - onboard_received_.header.stamp.toSec() < msg_timeout_.onboard;
    return is_received;
}

/**
 * @brief 判断降落目标位置是否接收到
 * @param now_time 时间
 * @return bool 是否接收到
 */
bool OnboardUavFsm::LandingTargetEskfIsReceived(const ros::Time &now_time)
{
    // TODO timeout 需要小于两倍的话题发布周期？这样会不会太短？
    //ROS_WARN("landing_target_eskf_ timeout: %f, %f", now_time.toSec(), landing_target_eskf_.header.stamp.toSec());
    bool is_received = now_time.toSec() - landing_target_eskf_.header.stamp.toSec() < msg_timeout_.landing_target;
    return is_received;
}

/**
 * @brief 判断母机指令是否为某个模式
 * @param flight_command 模式
 * @return bool 是否为某个模式
 */
bool OnboardUavFsm::IsOnboardCommand(const int &flight_command)
{
    bool is_mode = (onboard_received_.flight_command == flight_command);
    return is_mode;
}

/**
 * @brief 判断母机指令是否为某个状态
 * @param flight_status 状态
 * @return bool 是否为某个状态
 */
// bool OnboardUavFsm::IsOnboardStatus(const int &flight_status)
// {
//     bool is_status = (onboard_received_.flight_status == flight_status);
//     return is_status;
// }

/**
 * @brief 发布子机状态信息
 */
void OnboardUavFsm::PubOnboardMsg()
{
    onboard_published_.header.stamp = ros::Time::now();
    onboard_published_.header.frame_id = "map";
    onboard_published_.uav_id = onboard_uav_param_.uav_id;
    onboard_published_.target_uav_id = onboard_uav_param_.target_uav_id;
    onboard_msg_pub_.publish(onboard_published_);
}

/**
 * @brief 上锁线程函数
 */
void OnboardUavFsm::UavDisarm()
{
    mavros_msgs::CommandLong arm_cmd;
    arm_cmd.request.command = 400;
    arm_cmd.request.param1 = 0;
    arm_cmd.request.param2 = 21196;

    while (ros::ok())
    {
        if (perform_uav_disarm_)
        {
            if (arm_disarm_client_.call(arm_cmd) && arm_cmd.response.success)
            {
                // ROS_INFO("UAV disarmed");
            }
            else
            {
                ROS_ERROR("UAV disarm failed!");
            }
            // perform_uav_disarm_ = false;
        }
        ros::Duration(0.1).sleep();
    }

    // if (arm_disarm_client_.call(arm_cmd) && arm_cmd.response.success)
    // {
    //     ROS_INFO("UAV disarmed");
    //     return true;
    // }
    // else
    // {
    //     ROS_ERROR("UAV disarm failed!");
    //     return false;
    // }
}

/**
 * @brief 更新状态机
 * @param event 定时器事件
 */
void OnboardUavFsm::UpdataFsm(const ros::TimerEvent &event)
{
    // 心跳包
    static int fsm_count_heartbeat = 0;
    if (fsm_count_heartbeat++ >= 0.1 * fsm_hz_)
    {
        heartbeat_pub_.publish(std_msgs::Empty());
        fsm_count_heartbeat = 0;
    }

    // 发布子机状态信息
    PubOnboardUavState();

    // 打印状态
    static int fsm_count_print = 0;
    if (fsm_count_print++ >= 3 * fsm_hz_)
    {
        // PrintOnboardUavState();
        fsm_count_print = 0;
    }

    // 重规划
    static int fsm_count_replan = 0;
    static int replan_dt_count = fsm_hz_ / replan_hz_;
    if (fsm_count_replan >= replan_dt_count)
    {
        is_replan_ = true;
        fsm_count_replan = 0;
    }
    else
        fsm_count_replan++;
    

    ros::Time now_time = ros::Time::now();

    // 获取无人机位姿、速度
    static int fsm_count_odom = 0;
    if (OdomIsReceived(now_time))
    {
        fsm_count_odom = 0;
        uav_odom_pos_ << uav_local_pose_.pose.position.x, uav_local_pose_.pose.position.y, uav_local_pose_.pose.position.z;
        uav_odom_vel_ << uav_local_vel_.twist.linear.x, uav_local_vel_.twist.linear.y, uav_local_vel_.twist.linear.z;
        // uav_odom_orient_.x() = uav_local_pose_.pose.orientation.x;
        // uav_odom_orient_.y() = uav_local_pose_.pose.orientation.y;
        // uav_odom_orient_.z() = uav_local_pose_.pose.orientation.z;
        // uav_odom_orient_.w() = uav_local_pose_.pose.orientation.w;
    }
    else
    {
        fsm_count_odom++;
        if (fsm_count_odom >= 10 * fsm_hz_)
        {
            ROS_WARN("odom is not received!");
            // ROS_ERROR("odom is not received!");
            // onboard_uav_state_ = OnboardUavStates::FAIL_SAFE;
            // ROS_INFO("\033[32mSwitch to FAIL_SAFE\033[0m");
            fsm_count_odom = 0;
        }
        return;
    }

    FillLandingParams();
    traj_opt_ptr_->setLandingParams(land_params_);

    //调试使用
    // static int count = 0;
    // if (is_landing_target_vision_updated_ && count < 60) {
    //     first_frame_corrected_pos_.x = landing_target_vision_.pose.position.x;
    //     first_frame_corrected_pos_.y = landing_target_vision_.pose.position.y;
    //     first_frame_corrected_pos_.z = landing_target_vision_.pose.position.z;
    //     coord_align_pub_.publish(first_frame_corrected_pos_);
    //     count++;
    // }
    // eskf_active_.data = true;
    // eskf_actitve_pub_.publish(eskf_active_);
                             
    switch (onboard_uav_state_)
    {
        case OnboardUavStates::IDLE:
        {
            // 当接收到母机的起飞指令时，进入起飞状态
            if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::TAKEOFF))
            {
                hover_flag_ = false;
                onboard_uav_param_.real_takeoff_height = onboard_uav_param_.takeoff_height + uav_odom_pos_.z();
                onboard_uav_state_ = OnboardUavStates::TAKEOFF;
                ROS_INFO("\033[32mIDLE: Switch to TAKEOFF\033[0m");
            }
            break;
        }
        case OnboardUavStates::TAKEOFF:
        {
            // TODO 出现故障时，进入故障保护状态

            // 当接收到母机的任务指令时，进入任务状态
            if ((OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::MISSION)))
            {
                is_first_run_ = true;
                onboard_uav_state_ = OnboardUavStates::MISSION;
                ROS_INFO("\033[32mTAKEOFF: Switch to MISSION\033[0m");
                break;
            }
            // 当接收到母机的降落指令时，进入降落状态
            else if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::LAND))
            {
                onboard_uav_state_ = OnboardUavStates::LAND;
                ROS_INFO("\033[32mDOCKING: Switch to LAND\033[0m");
                break;
            }

            // 起飞完成后发送 Onboard::TAKEOFF_COMPLETE ，等待下一步指令
            if (uav_odom_pos_.z() >= onboard_uav_param_.real_takeoff_height - 0.3)
            {
                //onboard_published_.flight_command = quadrotor_msgs::Onboard::TAKEOFF;
                onboard_published_.flight_status = quadrotor_msgs::Onboard::TAKEOFF_COMPLETE;
                PubOnboardMsg();
                // 发布悬停
                if (!hover_flag_)
                {
                    // target_pos_ = uav_odom_pos_;
                    // target_pos_.z() += 0.1;
                    hover_flag_ = true;
                    // PubHoverPos();
                    ROS_INFO("TAKEOFF COMPLETE: Hover");
                }
                break;
            }

            // 发布起飞指令
            if (!uav_state_.armed && uav_state_.mode != "OFFBOARD")
            {
                quadrotor_msgs::TakeoffLand takeoff_land_cmd;
                takeoff_land_cmd.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
                takeoff_land_cmd_pub_.publish(takeoff_land_cmd);
            }
            break;
        }

        case OnboardUavStates::MISSION:
        {
            if (uav_state_.mode != "OFFBOARD")
            {
                break;
            }

            // 当接收到母机的远程引导指令时，进入引导返回状态
            if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::REMOTE_GUIDE))
            {
                onboard_uav_state_ = OnboardUavStates::REMOTE_GUIDE;
                ROS_INFO("\033[32mMISSION: Switch to REMOTE_GUIDANCE\033[0m");
                break;
            }
            // 当接收到母机的降落指令时，进入降落状态
            else if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::LAND))
            {
                onboard_uav_state_ = OnboardUavStates::LAND;
                ROS_INFO("\033[32mMISSION: Switch to LAND\033[0m");
                break;
            }

            RunMissionMode();
            break;
        }

        case OnboardUavStates::REMOTE_GUIDE:
        {
            // 当接收到母机的盘旋指令时，进入盘旋状态
            if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::SEARCH))
            {
                onboard_uav_state_ = OnboardUavStates::SEARCH;
                ROS_INFO("\033[32mMISSION: Switch to SEARCH\033[0m");
                break;
            }
            // 当接收到母机的盘旋指令时，进入盘旋状态
            if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::DOCKING))
            {
                docking_state_ = DockingStates::INIT;
                onboard_uav_state_ = OnboardUavStates::DOCKING;
                ROS_INFO("\033[32mMISSION: Switch to DOCKING\033[0m");
                break;
            }
            // 当接收到母机的降落指令时，进入降落状态
            else if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::LAND))
            {
                onboard_uav_state_ = OnboardUavStates::LAND;
                ROS_INFO("\033[32mMISSION: Switch to LAND\033[0m");
                break;
            }

            //调试使用
            // onboard_published_.flight_status = quadrotor_msgs::Onboard::REMOTE_GUIDE_COMPLETE;
            // //激活eskf
            // for (int i = 0; i < 10; ++i) {
            //     eskf_active_.data = true;
            //     eskf_actitve_pub_.publish(eskf_active_);
            // }
            // est_dock_world = uav_odom_pos_;
            // PubOnboardMsg();
            // break;

            Remote_Guidance();
            break;
        }

        case OnboardUavStates::SEARCH:
        {
            if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::DOCKING))
            {
                docking_state_ = DockingStates::INIT;
                onboard_uav_state_ = OnboardUavStates::DOCKING;
                ROS_INFO("\033[32mMISSION: Switch to DOCKING\033[0m");
                break;
            }
            // 当接收到母机的降落指令时，进入降落状态
            else if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::LAND))
            {
                onboard_uav_state_ = OnboardUavStates::LAND;
                ROS_INFO("\033[32mMISSION: Switch to LAND\033[0m");
                break;
            }

            //调试使用
            // onboard_published_.flight_status = quadrotor_msgs::Onboard::SEARCH_COMPLETE;
            // PubOnboardMsg();
            // break;

            Run_Search();
            break;
        }

        case OnboardUavStates::DOCKING:
        {
            // 当接收到母机的任务指令时，进入任务状态 没用到
            if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::MISSION))
            {
                is_first_run_ = true;
                onboard_uav_state_ = OnboardUavStates::MISSION;
                ROS_INFO("\033[32mDOCKING: Switch to MISSION\033[0m");
                break;
            }
            // 当接收到母机的降落指令时，进入降落状态 没用到
            else if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::LAND))
            {
                onboard_uav_state_ = OnboardUavStates::LAND;
                ROS_INFO("\033[32mDOCKING: Switch to LAND\033[0m");
                break;
            }

            RunDockingMode();
            break;
        }

        case OnboardUavStates::LAND:
        {
            // 通过 AUTO_LAND 进行降落
            static int fsm_count_land = 0;
            if (fsm_count_land++ > 0.6 * fsm_hz_)
            {
                quadrotor_msgs::TakeoffLand takeoff_land_cmd;
                takeoff_land_cmd.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
                takeoff_land_cmd_pub_.publish(takeoff_land_cmd);
                fsm_count_land = 0;
            }

            // 降落完成后发送 Onboard::LAND_COMPLETE
            if (!uav_state_.armed)
            {
                //onboard_published_.flight_command = quadrotor_msgs::Onboard::LAND;
                onboard_published_.flight_status = quadrotor_msgs::Onboard::LAND_COMPLETE;
                PubOnboardMsg();
                onboard_uav_state_ = OnboardUavStates::IDLE; // TODO bug
            }
            break;
        }

        case OnboardUavStates::FAIL_SAFE:
        {
            // 就地悬停
            static bool execute_once_flag = false;
            if (!execute_once_flag)
            {
                target_pos_ = uav_odom_pos_;
                PubHoverPos();
                execute_once_flag = true;
            }
            break;
        }

        default:
            break;
    }

    // update timer
    // fsm_dt_s_ = event.current_real.toSec() - event.last_real.toSec();
    Pub_FSM_State();

    return;
}

/**
 * @brief 运行任务模式
 */
void OnboardUavFsm::RunMissionMode()
{
    static int count = 0;
    static int idx = 0;

    const int mission_pt_count = sizeof(mission_pt) / sizeof(mission_pt[0]);

    // 执行任务
    ROS_WARN_THROTTLE(0.5, "RUN MISSION POINT: %d", idx);

    // 设置航点
    if (idx < mission_pt_count) {
        target_pos_ = mission_pt[idx];
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_ = Eigen::Quaterniond::Identity();
    }
    hover_flag_ = false;
    ROS_WARN_THROTTLE(0.5, "target_pos_ : %f,%f,%f",target_pos_.x(),target_pos_.y(),target_pos_.z());
    // DEBUG 打印目标位姿

    if ((uav_odom_pos_ - target_pos_).norm() < 0.3)
    {
        count ++;
        if(count > 20){
            if (idx == mission_pt_count-1) {  
                if (mother_ready) {
                    onboard_published_.flight_status = quadrotor_msgs::Onboard::MISSION_COMPLETE;
                    PubOnboardMsg();
                    count = 0;
                    idx = 0;
                    return;
                } else {
                    count = 0; 
                    return;
                }
            } else {
                count = 0;
                idx ++;
            }
        }
    }

    // 调用规划器，规划任务轨迹
    if (!hover_flag_ && PlanTrajectory())
    {
        is_replan_ = false;
    }

    else
        return;
}

void OnboardUavFsm::Remote_Guidance()
{
    Pub_Guidance_State();

    static int settle_cnt = 0;
    static int mean_cnt   = 0;

    static std::vector<double> r_buf;
    static std::vector<Eigen::Vector3d> p_buf_world;

    if (!rg_first_ref_inited_) {
        rg_first_ref_world_ = uav_odom_pos_;
        rg_first_ref_inited_ = true;
        ROS_INFO("[RG] rg_first_ref_world_ (from uav_odom_pos_)=(%.3f,%.3f,%.3f)",
                 rg_first_ref_world_.x(), rg_first_ref_world_.y(), rg_first_ref_world_.z());
    }

    if (rg_stage_ == 2 && remote_guide_param_.fly_away_test) {
        std_msgs::Int32 mother_move_msg;
        mother_move_msg.data = 1;
        mother_move_pub_.publish(mother_move_msg);
    }

    if (!rg_base_inited_) {
        rg_base_inited_ = true;

        rg_base_pos_local_.setZero();
        rg_stage_ = 0;

        settle_cnt = 0;
        mean_cnt   = 0;
        r_buf.clear();
        p_buf_world.clear();

        rg_anchors_win_.clear();

        rg_have_solution_ = false;
        rg_last_rms_res_  = std::numeric_limits<double>::infinity();
        rg_last_max_res_  = std::numeric_limits<double>::infinity();

        rg_have_mother_world_ = false;
        rg_last_mother_world_.setZero();

        ROS_INFO("[RG] init base_pos_local=(0,0,0), stage=0");
    }

    auto pattern_local = [&](int i)->Eigen::Vector3d {
        switch(i){
            case 0: return Eigen::Vector3d(remote_guide_param_.p_set_1_x,
                                           remote_guide_param_.p_set_1_y,
                                           remote_guide_param_.p_set_1_z);
            case 1: return Eigen::Vector3d(remote_guide_param_.p_set_2_x,
                                           remote_guide_param_.p_set_2_y,
                                           remote_guide_param_.p_set_2_z);
            case 2: return Eigen::Vector3d(remote_guide_param_.p_set_3_x,
                                           remote_guide_param_.p_set_3_y,
                                           remote_guide_param_.p_set_3_z);
            case 3: return Eigen::Vector3d(remote_guide_param_.p_set_4_x,
                                           remote_guide_param_.p_set_4_y,
                                           remote_guide_param_.p_set_4_z);
            case 4: return Eigen::Vector3d(remote_guide_param_.p_set_5_x,
                                           remote_guide_param_.p_set_5_y,
                                           remote_guide_param_.p_set_5_z);
            case 5: return Eigen::Vector3d(remote_guide_param_.p_set_6_x,
                                           remote_guide_param_.p_set_6_y,
                                           remote_guide_param_.p_set_6_z);
            case 6: return Eigen::Vector3d(remote_guide_param_.p_set_7_x,
                                           remote_guide_param_.p_set_7_y,
                                           remote_guide_param_.p_set_7_z);
            case 7: return Eigen::Vector3d(remote_guide_param_.p_set_8_x,
                                           remote_guide_param_.p_set_8_y,
                                           remote_guide_param_.p_set_8_z);
            default: return Eigen::Vector3d(0, 0, 0);
        }
    };

    const Eigen::Vector3d dock_r2m(remote_guide_param_.dock_r2m_x,
                                   remote_guide_param_.dock_r2m_y,
                                   remote_guide_param_.dock_r2m_z);

    auto solve_and_step_from_anchors = [&]() -> bool
    {
        if ((int)rg_anchors_win_.size() < remote_guide_param_.win_anchor_n) return false;

        Eigen::Vector3d mother_local, mother_world;
        double max_res = 0.0, rms_res = 0.0;

        if (!estimate_mother_from_window(rg_anchors_win_, mother_local, max_res, rms_res)) {
            ROS_WARN("[RG] estimate failed.");
            rg_have_solution_ = false;
            rg_last_max_res_  = std::numeric_limits<double>::infinity();
            rg_last_rms_res_  = std::numeric_limits<double>::infinity();

            rg_have_mother_world_ = false;
            return false;
        }

        rg_have_solution_ = true;
        rg_last_max_res_  = max_res;
        rg_last_rms_res_  = rms_res;

        mother_world = rg_first_ref_world_ + mother_local;

        rg_last_mother_world_ = mother_world;
        rg_have_mother_world_ = true;

        const Eigen::Vector3d dock_world = mother_world + dock_r2m;  // mother->dock
        const Eigen::Vector3d dock_local = dock_world - rg_first_ref_world_;

        ROS_INFO("[RG] solve@anchors: mother_world=(%.3f,%.3f,%.3f) dock_world=(%.3f,%.3f,%.3f) max_res=%.3f rms=%.3f",
                 mother_world.x(), mother_world.y(), mother_world.z(),
                 dock_world.x(), dock_world.y(), dock_world.z(),
                 max_res, rms_res);

        Eigen::Vector3d dir = dock_local - rg_base_pos_local_;
        double dist_to_dock = dir.norm();
        if (dist_to_dock > 1e-3) {
            dir /= dist_to_dock;

            double step = remote_guide_param_.step_gamma * dist_to_dock;
            step = std::max(remote_guide_param_.step_min,
                            std::min(step, remote_guide_param_.step_max));

            const bool pass_gate =
                (max_res < remote_guide_param_.res_gate_max) &&
                (rms_res < remote_guide_param_.res_gate_rms);

            if (pass_gate) {
                rg_base_pos_local_ += step * dir;
                ROS_INFO("[RG] base update(to dock): step=%.3f -> base_local=(%.3f,%.3f,%.3f)",
                         step, rg_base_pos_local_.x(), rg_base_pos_local_.y(), rg_base_pos_local_.z());
            } else {
                ROS_WARN("[RG] skip base update: max_res=%.3f (gate %.3f), rms=%.3f (gate %.3f)",
                         max_res, remote_guide_param_.res_gate_max,
                         rms_res, remote_guide_param_.res_gate_rms);
            }
        }
        return true;
    };

    const Eigen::Vector3d sp_local = rg_base_pos_local_ + pattern_local(rg_stage_);
    const Eigen::Vector3d sp_world = rg_first_ref_world_ + sp_local;


    target_pos_ = sp_world;
    target_vel_ = Eigen::Vector3d::Zero();
    target_q_   = Eigen::Quaterniond::Identity();
    hover_flag_ = false;

    if (PlanTrajectory()) {
        is_replan_ = false;
    }

    const double reach_thr  = std::max(0.05, remote_guide_param_.reach_target_thr);
    const double dist_to_sp = (uav_odom_pos_ - sp_world).norm();

    const int need_settle = std::max(1, remote_guide_param_.settle_cnt);
    const int need_mean   = std::max(1, remote_guide_param_.mean_cnt);

    bool formed_anchor = false;

    if (dist_to_sp < reach_thr) {

        settle_cnt++;

        if (settle_cnt < need_settle) {

            mean_cnt = 0;
            r_buf.clear();
            p_buf_world.clear();

            ROS_INFO_THROTTLE(0.2,
                "[RG] stage=%d settling %d/%d dist_to_sp=%.3f (no sampling yet)",
                rg_stage_, settle_cnt, need_settle, dist_to_sp);

        } else {

            r_buf.push_back(uwb_distance);
            p_buf_world.push_back(uav_odom_pos_);
            mean_cnt++;

            ROS_INFO_THROTTLE(0.2,
                "[RG] stage=%d holding %d/%d uwb=%.3f dist_to_sp=%.3f",
                rg_stage_, mean_cnt, need_mean, uwb_distance, dist_to_sp);

            if (mean_cnt >= need_mean) {

                double sum = 0.0;
                for (double v : r_buf) sum += v;
                const double r_mean = sum / std::max(1, (int)r_buf.size());

                Eigen::Vector3d p_sum = Eigen::Vector3d::Zero();
                for (const auto& pw : p_buf_world) p_sum += pw;
                Eigen::Vector3d p_mean_world = p_sum / std::max(1, (int)p_buf_world.size());
                Eigen::Vector3d p_mean_local = p_mean_world - rg_first_ref_world_;

                rg_sample_pos_ = p_mean_world;
                RangeSample a;
                a.p_local = p_mean_local;
                a.r       = r_mean;
                a.stamp   = ros::Time::now();

                rg_anchors_win_.push_back(a);
                while ((int)rg_anchors_win_.size() > remote_guide_param_.win_anchor_n)
                    rg_anchors_win_.pop_front();

                ROS_INFO("[RG] add anchor: win=%zu (need %d) stage=%d p_local=(%.3f,%.3f,%.3f) r_mean=%.3f",
                         rg_anchors_win_.size(), remote_guide_param_.win_anchor_n, rg_stage_,
                         a.p_local.x(), a.p_local.y(), a.p_local.z(), a.r);

                r_buf.clear();
                p_buf_world.clear();
                mean_cnt = 0;
                settle_cnt = 0;

                rg_stage_ = (rg_stage_ + 1) % remote_guide_param_.win_anchor_n;
                formed_anchor = true;
            }
        }

    } else {
        settle_cnt = 0;
        mean_cnt = 0;
        r_buf.clear();
        p_buf_world.clear();
    }

    if (formed_anchor && (int)rg_anchors_win_.size() == remote_guide_param_.win_anchor_n) {
        solve_and_step_from_anchors();
    }

    double dock_err = std::numeric_limits<double>::infinity();
    Eigen::Vector3d dock_world = Eigen::Vector3d::Zero();

    if (rg_have_mother_world_) {
        dock_world = rg_last_mother_world_ + dock_r2m;
        dock_err = (uav_odom_pos_ - dock_world).norm();
    }

    const bool pass_pos = (rg_have_mother_world_ && std::isfinite(dock_err) &&
                           dock_err < remote_guide_param_.exit_dist_thr);

    const bool pass_res = (rg_have_solution_ && std::isfinite(rg_last_rms_res_) &&
                           rg_last_rms_res_ < remote_guide_param_.exit_res_thr);

    ROS_INFO_THROTTLE(0.5,
        "[RG] exit check: dock_err=%.3f (%s %.3f) | rms=%.3f (%s %.3f) | last_max=%.3f | have_solution=%d | have_mother=%d",
        dock_err, (pass_pos ? "<" : ">="), remote_guide_param_.exit_dist_thr,
        rg_last_rms_res_, (pass_res ? "<" : ">="), remote_guide_param_.exit_res_thr,
        rg_last_max_res_, (int)rg_have_solution_, (int)rg_have_mother_world_
    );

    ROS_INFO_THROTTLE(0.5,
        "[RG] uav_world=(%.3f,%.3f,%.3f) dock_world=(%.3f,%.3f,%.3f)",
        uav_odom_pos_.x(), uav_odom_pos_.y(), uav_odom_pos_.z(),
        dock_world.x(), dock_world.y(), dock_world.z()
    );

    if (pass_pos && pass_res) {
        onboard_published_.flight_status = quadrotor_msgs::Onboard::REMOTE_GUIDE_COMPLETE;
        //激活eskf
        for (int i = 0; i < 10; ++i) {
            eskf_active_.data = true;
            eskf_actitve_pub_.publish(eskf_active_);
        }
        est_dock_world = dock_world;
        PubOnboardMsg();
        ROS_INFO("[RG] REMOTE GUIDANCE COMPLETE");
        return;
    }
}


bool OnboardUavFsm::estimate_mother_from_window(
    const std::deque<RangeSample>& win,
    Eigen::Vector3d& mother_local,
    double& max_residual,
    double& rms_residual)
{
    const int N = (int)win.size();
    if (N < remote_guide_param_.win_anchor_n) return false;

    thread_local bool has_prev = false;
    thread_local Eigen::Vector3d P_prev = Eigen::Vector3d::Zero();

    Eigen::Vector3d P = Eigen::Vector3d::Zero();

    if (has_prev) {
        P = P_prev;
    } else {
        // 线性初值：2(p0 - pi)^T X = r_i^2 - r_0^2 + ||p_i||^2 - ||p_0||^2
        const Eigen::Vector3d p0 = win[0].p_local;
        const double r0 = win[0].r;

        Eigen::MatrixXd A(N - 1, 3);
        Eigen::VectorXd b(N - 1);

        const double p0_sq = p0.squaredNorm();
        for (int i = 1; i < N; ++i) {
            const Eigen::Vector3d pi = win[i].p_local;
            const double ri = win[i].r;

            A.row(i - 1) = (2.0 * (p0 - pi)).transpose();
            b(i - 1) = (ri * ri - r0 * r0) + (pi.squaredNorm() - p0_sq);
        }

        Eigen::Vector3d X0 = A.colPivHouseholderQr().solve(b);
        if (!X0.allFinite()) return false;

        P = X0;
        has_prev = true;
        P_prev = P;
    }

    // ---------- Huber 权重 ----------
    auto huber_weight = [](double abs_r, double delta) {
        if (abs_r <= delta) return 1.0;
        return delta / abs_r;
    };

    // 这个值建议设置为“正常 UWB 噪声的 2~3 倍”
    const double huber_delta = 0.5;

    auto robust_cost = [&](const Eigen::Vector3d& X) {
        double s = 0.0;
        for (int i = 0; i < N; ++i) {
            const double ri = (X - win[i].p_local).norm() - win[i].r;
            const double wi = huber_weight(std::abs(ri), huber_delta);
            s += wi * ri * ri; // 加权平方残差
        }
        return s;
    };

    // ---------- LM ----------
    double lambda = 1e-3;
    double c0 = robust_cost(P);

    for (int iter = 0; iter < 15; ++iter) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d g = Eigen::Vector3d::Zero();

        for (int i = 0; i < N; ++i) {
            Eigen::Vector3d d = P - win[i].p_local;
            double dist = d.norm();
            if (dist < 1e-8) dist = 1e-8;

            const double ri = dist - win[i].r;           // residual
            const double wi = huber_weight(std::abs(ri), huber_delta);

            const Eigen::Vector3d Ji = d / dist;         // 3x1

            H.noalias() += wi * (Ji * Ji.transpose());
            g.noalias() += wi * (Ji * ri);
        }

        // 病态保护：对角线太小就加大阻尼
        const double min_diag = H.diagonal().minCoeff();
        if (!std::isfinite(min_diag)) return false;
        if (min_diag < 1e-10) lambda = std::min(1e2, lambda * 10.0);

        // (H + lambda I) dx = -g
        Eigen::Matrix3d H_lm = H + lambda * Eigen::Matrix3d::Identity();
        Eigen::LDLT<Eigen::Matrix3d> ldlt(H_lm);
        if (ldlt.info() != Eigen::Success) {
            lambda = std::min(1e2, lambda * 10.0);
            continue;
        }

        Eigen::Vector3d dx = ldlt.solve(-g);
        if (!dx.allFinite()) return false;

        // 大步保护：避免一次跳太远
        if (dx.norm() > 10.0) {
            lambda = std::min(1e2, lambda * 10.0);
            continue;
        }

        const Eigen::Vector3d P_new = P + dx;
        const double c1 = robust_cost(P_new);

        if (std::isfinite(c1) && c1 < c0) {
            P = P_new;
            c0 = c1;
            lambda = std::max(1e-6, lambda * 0.5);

            if (dx.norm() < 1e-4) break;
        } else {
            lambda = std::min(1e2, lambda * 5.0);
        }
    }

    mother_local = P;
    P_prev = P;
    has_prev = true;

    max_residual = 0.0;
    double sum2 = 0.0;
    for (int i = 0; i < N; ++i) {
        const double pred = (P - win[i].p_local).norm();
        const double res  = std::abs(pred - win[i].r);
        max_residual = std::max(max_residual, res);
        sum2 += res * res;
    }
    rms_residual = std::sqrt(sum2 / N);

    return std::isfinite(max_residual) && std::isfinite(rms_residual);
}


bool OnboardUavFsm::Circle_Search() {
    static bool detect = false, cir_done = false;
    static Eigen::Vector3d cir_center = uav_odom_pos_;
    static double cir_radius = 0.2;   
    static double start_time = ros::Time::now().toSec();

    const double v = 0.3; 

    double t = ros::Time::now().toSec() - start_time;  
    double omega = v / cir_radius;                     
    double T = 2.0 * M_PI / omega;                   
    double theta = t * omega;

    target_pos_ << cir_center[0] + cir_radius * cos(theta), 
                   cir_center[1] + cir_radius * sin(theta), 
                   cir_center[2];

    if (PlanTrajectory())
    {
        is_replan_ = false;
    }

    if (is_landing_target_vision_updated_ && is_landing_target_eskf_updated_) {
        circle_search_target.x() = landing_target_eskf_.pose.position.x;
        circle_search_target.y() = landing_target_eskf_.pose.position.y;
        first_frame_corrected_pos_.x = landing_target_vision_.pose.position.x;
        first_frame_corrected_pos_.y = landing_target_vision_.pose.position.y;
        first_frame_corrected_pos_.z = landing_target_vision_.pose.position.z;
        detect = true;
    }

    if (t >= T) {
        cir_done = true;
        cir_radius += 0.2;                    
        start_time = ros::Time::now().toSec(); 
    }

    return (detect && cir_done);
}


void OnboardUavFsm::Run_Search() {

    if(remote_guide_param_.search_flag){
        bool find_tag = Circle_Search();
        if(find_tag){
            remote_guide_param_.search_flag = false;
        }
        else{
            ROS_INFO("CIRCIE SEARCHING");
        }
    }

    else{
        for(int i = 0; i < 10; ++i) {
            coord_align_pub_.publish(first_frame_corrected_pos_);
        }
        onboard_published_.flight_status = quadrotor_msgs::Onboard::SEARCH_COMPLETE;
        PubOnboardMsg();
        ROS_INFO("SEARCH COMPLETE");
    }
}

/**
 * @brief 对接状态机
 */
void OnboardUavFsm::RunDockingMode()
{
    switch (docking_state_)
    {
        case DockingStates::INIT:
        {
            RunDockingIdle();
            break;
        }

        case DockingStates::RETURN:
        {
            RunDockingReturn();
            break;
        }

        case DockingStates::LANDING:
        {   
            RunDockingLanding();
            break;
        }

        case DockingStates::COMPLETE:
        {
            RunDockingComplete();
            break;
        }
        case DockingStates::RETRY:
        {
            RunDockingRetry();
            break;
        }

        default:
            break;
    }
    return;
}

/**
 * @brief 对接状态机：空闲状态
 * @details 当接收到母机的返航指令时，进入返航状态；否则悬停
 * @details 来源：OnboardUavStates::MISSION、DockingStates::RETRY
 */
void OnboardUavFsm::RunDockingIdle()
{
    // 当接收到母机的返航指令时，进入返航状态
    if ((OnboardMsgIsReceived(ros::Time::now()) && IsOnboardCommand(quadrotor_msgs::Onboard::ALLOW_RETURN)) || is_docking_retry_)
    {
        hover_flag_ = false;
        is_first_run_ = true;
        //调试使用
        // docking_state_ = DockingStates::LANDING;
        docking_state_ = DockingStates::RETURN;
        ROS_INFO("\033[32mDOCKING_IDLE: Switch to DOCKING RETURN\033[0m");
    }
    else
    {
        //onboard_published_.flight_command = quadrotor_msgs::Onboard::DOCKING;
        onboard_published_.flight_status = quadrotor_msgs::Onboard::SEARCH_COMPLETE;
        PubOnboardMsg();
        // 发布悬停
        if (!hover_flag_)
        {
            // target_pos_ = uav_odom_pos_;
            hover_flag_ = true;
            // PubHoverPos();
            ROS_INFO("DOCKING_IDLE: Hover");
        }
    }
}

/**
 * @brief 对接状态机：返航状态
 * @details 当接收到母机的降落指令，且标签可见时，进入对接降落状态；否则到达新返航位置，悬停
 * @details 来源：DockingStates::IDLE
 */
void OnboardUavFsm::RunDockingReturn()
{
    // 设置返航目标点
    if (is_first_run_ && !is_docking_retry_)
    {
        target_pos_.x() = circle_search_target.x();
        target_pos_.y() = circle_search_target.y();
        target_pos_.z() = est_dock_world.z(); //2026110todo
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_ = Eigen::Quaterniond::Identity();
    }

    // 当接收到母机的降落指令，且标签可见时，进入降落状态，
    // 标签不可见时，进一步到达返航位置，仍然不可见时，悬停，此时还会发送 Onboard::RETURN_COMPLETE
    // TODO 让母机在重试时也发送降落指令
    if ((OnboardMsgIsReceived(ros::Time::now()) && IsOnboardCommand(quadrotor_msgs::Onboard::ALLOW_PRECISION_LANDING)) || is_docking_retry_)
    // if (IsOnboardCommand(quadrotor_msgs::Onboard::ALLOW_PRECISION_LANDING))
    {
        if (LandingTargetEskfIsReceived(ros::Time::now()))
        {
            is_first_run_ = true;
            landing_state_ = LandingStates::INIT;
            docking_state_ = DockingStates::LANDING;
            ROS_INFO("\033[32mDOCKING_RETURN: Switch to LANDING\033[0m");
        }
        else
        {
            // 当接收到母机的降落指令，但标签不可见时，返航至新的指定位置
            is_replan_ = true;
            target_pos_.x() = circle_search_target.x();
            target_pos_.y() = circle_search_target.y();
            target_pos_.z() = est_dock_world.z(); //2026110todo
            target_vel_ = Eigen::Vector3d::Zero();
            target_q_ = Eigen::Quaterniond::Identity();
            ROS_INFO("Receive Down Cmd, But cannot find tag");
            // 当到达指定返航位置时，悬停
            if ((uav_odom_pos_ - target_pos_).norm() < 0.3)
            {
                // 发布悬停
                if (!hover_flag_)
                {
                    hover_flag_ = true;
                    ROS_WARN("DOCKING_RETURN: Landing target lost! Hover");
                }
            }
            else // if (!hover_flag_)
            {
                ROS_WARN("DOCKING_RETURN: Landing target lost!");
                // 调用规划器，规划返航轨迹
                if (PlanTrajectory())
                {
                    is_replan_ = false;
                    hover_flag_ = false;
                    // DEBUG 打印目标位姿
                    //PrintUAVPosVel();
                }
            }
            return;
        }
    }

    // 当到达指定返航位置时，发送 Onboard::RETURN_COMPLETE，等待下一步指令
    if ((uav_odom_pos_ - target_pos_).norm() < 0.15)
    {
        //onboard_published_.flight_command = quadrotor_msgs::Onboard::DOCKING;
        onboard_published_.flight_status = quadrotor_msgs::Onboard::RETURN_COMPLETE;
        PubOnboardMsg();

        // 发布悬停
        if (!hover_flag_)
        {
            // target_pos_ = uav_odom_pos_;
            hover_flag_ = true;
            // PubHoverPos();
            ROS_INFO("DOCKING_RETURN: Hover");
        }
        return;
    }

    // 调用规划器，规划返航轨迹
    if (!hover_flag_ && PlanTrajectory())
    {
        is_replan_ = false;
    }
}

/**
 * @brief 对接状态机：降落状态
 * @details 当标签不可见，且不是 FINAL_LANDING 时，进入重试状态；否则执行降落状态机
 */
void OnboardUavFsm::RunDockingLanding()
{
    if (!LandingTargetEskfIsReceived(ros::Time::now()))
    {
        retry_state_ = RetryStates::INIT;
        docking_state_ = DockingStates::RETRY;
        ROS_WARN("DOCKING_LANDING: Landing target lost! Switch to RETRY");
        is_landing_ = false;
        FillLandingParams();
        traj_opt_ptr_->setLandingParams(land_params_);
        return;
    }

    is_landing_ = true;
    FillLandingParams();
    traj_opt_ptr_->setLandingParams(land_params_);

    // 计算无人机与标签的水平距离
    static Eigen::Vector3d relative_pos = Eigen::Vector3d::Zero();
    static double horizontal_distance = 0.0;
    relative_pos.x() = landing_target_eskf_.pose.position.x - uav_odom_pos_.x();
    relative_pos.y() = landing_target_eskf_.pose.position.y - uav_odom_pos_.y();
    relative_pos.z() = landing_target_eskf_.pose.position.z - uav_odom_pos_.z();
    
    ROS_INFO("relative_pos:%f,%f,%f",relative_pos.x(),relative_pos.y(),relative_pos.z());
    horizontal_distance = relative_pos.head(2).norm();

    switch (landing_state_)
    {
        case LandingStates::INIT:
        {
            is_first_run_ = true;
            // 调试使用
            //landing_state_ = LandingStates::FINAL_LANDING;
            landing_state_ = LandingStates::DESCEND_ABOVE_TARGET;
            ROS_INFO("\033[32mDOCKING_INIT: Switch to DESCEND_ABOVE_TARGET\033[0m");
            break;
        }

        case LandingStates::DESCEND_ABOVE_TARGET:
        {
            static int landing_descend_complete_count = 0;

            if (std::fabs(relative_pos.z()) < docking_param_.descend_ver_bias + docking_param_.z_control_dead && horizontal_distance < docking_param_.descend_hor_bias)
            {
                landing_descend_complete_count++;
                if(landing_descend_complete_count > 5)
                {
                    first_frame_corrected_pos_.x = landing_target_vision_.pose.position.x;
                    first_frame_corrected_pos_.y = landing_target_vision_.pose.position.y;
                    first_frame_corrected_pos_.z = landing_target_vision_.pose.position.z;
                    for(int i = 0; i < 10; ++i) {
                        coord_align_pub_.publish(first_frame_corrected_pos_);
                    }
                    landing_state_ = LandingStates::RE_CURRATE;
                    ROS_INFO("\033[32mDOCKING_DESCEND_ABOVE_TARGET: Switch to RE_CURRATE\033[0m");
                    break;
                }
            }
            else
            {
                landing_descend_complete_count = 0;
            }

            target_pos_.x() = landing_target_eskf_.pose.position.x;
            target_pos_.y() = landing_target_eskf_.pose.position.y;
            target_pos_.z() = landing_target_eskf_.pose.position.z + docking_param_.descend_ver_bias;
            target_vel_ = Eigen::Vector3d::Zero();
            target_q_.x() = landing_target_eskf_.pose.orientation.x;
            target_q_.y() = landing_target_eskf_.pose.orientation.y;
            target_q_.z() = landing_target_eskf_.pose.orientation.z;
            target_q_.w() = landing_target_eskf_.pose.orientation.w;

            if (PlanTrajectory()) {
                is_replan_ = false;
                // 打印目标位姿
                //ROS_INFO("des:TARGET_POS:%.2f,%.2f,%.2f\n",target_pos_.x(),target_pos_.y(),target_pos_.z());
            }
            
            break;
        }

        case LandingStates::RE_CURRATE:
        {
            static int landing_rec_count = 0;

            if (std::fabs(relative_pos.z()) < docking_param_.rec_ver_bias + docking_param_.z_control_dead && horizontal_distance < docking_param_.rec_hor_bias)
            {
                landing_rec_count++;
                if(landing_rec_count > 5)
                {
                    first_frame_corrected_pos_.x = landing_target_vision_.pose.position.x;
                    first_frame_corrected_pos_.y = landing_target_vision_.pose.position.y;
                    first_frame_corrected_pos_.z = landing_target_vision_.pose.position.z;
                    for(int i = 0; i < 10; ++i) {
                        coord_align_pub_.publish(first_frame_corrected_pos_);
                    }
                    landing_state_ = LandingStates::FINAL_LANDING;
                    ROS_INFO("\033[32mDOCKING_RE_CURRATE: Switch to FINAL_LANDING\033[0m");
                    break;
                }
            }
            else
            {
                landing_rec_count = 0;
            }

            target_pos_.x() = landing_target_eskf_.pose.position.x;
            target_pos_.y() = landing_target_eskf_.pose.position.y;
            target_pos_.z() = landing_target_eskf_.pose.position.z + docking_param_.rec_ver_bias;
            target_vel_ = Eigen::Vector3d::Zero();
            target_q_.x() = landing_target_eskf_.pose.orientation.x;
            target_q_.y() = landing_target_eskf_.pose.orientation.y;
            target_q_.z() = landing_target_eskf_.pose.orientation.z;
            target_q_.w() = landing_target_eskf_.pose.orientation.w;

            if (PlanTrajectory())
            {
                is_replan_ = false;
                // 打印目标位姿
                //ROS_INFO("re:TARGET_POS:%.2f,%.2f,%.2f\n",target_pos_.x(),target_pos_.y(),target_pos_.z());
            }
            break;
        }

        case LandingStates::FINAL_LANDING:
        {
            // 当无人机与标签的垂直距离小于降落完成高度，且水平距离小于允许降落误差时，进入降落完成状态
            static int landing_complete_count = 0;
            static int landing_horizon_error = 0;
            if (std::fabs(relative_pos.z()) < docking_param_.final_ver_bias + docking_param_.z_control_dead && horizontal_distance < docking_param_.final_hor_bias)
            {
                landing_complete_count++;
                if (landing_complete_count > 3)
                {
                    docking_state_ = DockingStates::COMPLETE;
                    ROS_INFO("\033[32mDOCKING_FINAL_LANDING: Switch to COMPLETE\033[0m");
                    break;
                }
            }
            else
            {
                landing_complete_count = 0;
            }

            //如果高度已经达到目标点但水平差距较大，此时小飞机在大飞机平板上由于摩擦力很难再进行水平矫正，进入重试阶段
            if (std::fabs(relative_pos.z()) < docking_param_.final_ver_bias + docking_param_.z_control_dead && horizontal_distance > 0.18f)
            {
                landing_horizon_error ++;
                if(landing_horizon_error > 3)
                {
                    retry_state_ = RetryStates::INIT;
                    docking_state_ = DockingStates::RETRY;
                    ROS_INFO("Horizon_bias_error,switch to retry state");
                    break;
                }
            }
            else
            {
                landing_horizon_error = 0;
            }

            // 当目标点更新时，重新规划轨迹
            if (is_landing_target_eskf_updated_)
            {
                // 调试使用
                // is_replan_ = true;
                // target_pos_.x() = 3.0;
                // target_pos_.y() = -4.0;
                // target_pos_.z() = 2.15;
                // target_vel_ = Eigen::Vector3d::Zero();
                // target_q_ = Eigen::Quaterniond::Identity();

                is_replan_ = true;
                target_pos_.x() = landing_target_eskf_.pose.position.x;
                target_pos_.y() = landing_target_eskf_.pose.position.y;
                target_pos_.z() = landing_target_eskf_.pose.position.z;
                target_vel_ = Eigen::Vector3d::Zero();
                target_q_.x() = landing_target_eskf_.pose.orientation.x;
                target_q_.y() = landing_target_eskf_.pose.orientation.y;
                target_q_.z() = landing_target_eskf_.pose.orientation.z;
                target_q_.w() = landing_target_eskf_.pose.orientation.w;
            }

            //如果最终降落阶段没识别到二维码，则抬高高度
            //如果没识别到不会进入这里，直接进入retry
            else
            {
                target_pos_.z() = uav_odom_pos_.z() + 0.25;
                ROS_INFO("LOSE TARGET--- DANGER");
            }

            if (PlanTrajectory()) {
                is_replan_ = false;
                // 打印目标位姿
                // ROS_INFO("fin:TARGET_POS:%.2f,%.2f,%.2f\n",target_pos_.x(),target_pos_.y(),target_pos_.z());
            }

            break;
        }
        default:
            break;
    }
}

void OnboardUavFsm::RunDockingComplete()
{
    // 发送 Onboard::LAND_COMPLETE
    //onboard_published_.flight_command = quadrotor_msgs::Onboard::DOCKING;
    onboard_published_.flight_status = quadrotor_msgs::Onboard::PRECISION_LANDING_COMPLETE;
    PubOnboardMsg();

    // TODO 发送信号给traj_server，让其停止发布，停止发布心跳包
    if (uav_state_.armed == false)
    {
        perform_uav_disarm_ = false;
        ROS_WARN("\033[32mDOCKING_COMPLETE: Docking complete! UAV disarmed!\033[0m");
        onboard_uav_state_ = OnboardUavStates::IDLE;
    }
    else
    {
        perform_uav_disarm_ = true;
    }
    return;
}

void OnboardUavFsm::RunDockingRetry()
{
    // TODO 给定信号给docking，不然还得等待母机信号
    is_docking_retry_ = true;

    switch (retry_state_)
    {
        case RetryStates::INIT:
        {
            hover_flag_ = false;
            is_first_run_ = true;
            retry_start_time_ = ros::Time::now();
            retry_state_ = RetryStates::HOVER_SEARCH;
            ROS_INFO("\033[32mDOCKING_RETRY: Switch to HOVER_SEARCH\033[0m");
            break;
        }
        case RetryStates::HOVER_SEARCH:
        {
            // 当再次获取到降落目标位置时，重新进入降落状态
            if (LandingTargetEskfIsReceived(ros::Time::now()))
            {
                ROS_INFO("\033[32mDOCKING_RETRY: Target detected!\033[0m");
                is_first_run_ = true;
                landing_state_ = LandingStates::DESCEND_ABOVE_TARGET;
                docking_state_ = DockingStates::LANDING;
                ROS_INFO("\033[32mDOCKING_RETRY: Switch to DOCKING LANDING\033[0m");
                break;
            }
            // 当重试悬停超时时，进入重试爬升状态，重试次数减一
            if ((ros::Time::now() - retry_start_time_).toSec() > docking_param_.allowed_retry_hover_time_s)
            {
                is_first_run_ = true;
                docking_param_.allowed_retry_num--;

                target_pos_ = uav_odom_pos_;
                target_pos_.z() += docking_param_.retry_climb_height;
                target_vel_ = Eigen::Vector3d::Zero();
                target_q_ = Eigen::Quaterniond::Identity(); // TODO 当前姿态
                retry_state_ = RetryStates::CLIMBING;
                ROS_INFO("\033[32mDOCKING_RETRY: Switch to CLIMBING\033[0m");
                // DEBUG 打印目标位姿
                //PrintUAVPosVel();
                break;
            }

            target_pos_ = uav_odom_pos_;

            if (PlanTrajectory()) {
                is_replan_ = false;
                // 打印目标位姿
                // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
            }
            ROS_INFO("DOCKING_RETRY: Hover");

            break;
        }

        case RetryStates::CLIMBING:
        {
            // 当再次获取到降落目标位置时，重新进入降落状态
            if (LandingTargetEskfIsReceived(ros::Time::now()))
            {
                ROS_INFO("\033[32mDOCKING_RETRY: Target detected!\033[0m");
                is_first_run_ = true;
                landing_state_ = LandingStates::DESCEND_ABOVE_TARGET;
                docking_state_ = DockingStates::LANDING;
                ROS_INFO("\033[32mDOCKING_RETRY: Switch to DOCKING LANDING\033[0m");
                break;
            }

            // 当重试次数为零时，进入故障保护状态
            if (docking_param_.allowed_retry_num <= 0)
            {
                ROS_ERROR("DOCKING_RETRY: Allowed retry num used up!");
                onboard_uav_state_ = OnboardUavStates::FAIL_SAFE;
                ROS_INFO("\033[32mDOCKING_RETRY: Switch to FAIL_SAFE\033[0m");
                break;
            }

            // 当重试爬升到指定位置时，重新进入远程引导状态
            if ((uav_odom_pos_ - target_pos_).norm() < 0.3)
            {
                // 使用返航位置，母机指令中允许返航以及允许精准降落中的位置一样
                target_pos_.x() = onboard_received_.position.x - onboard_uav_param_.origin_pos_offset[0];
                target_pos_.y() = onboard_received_.position.y - onboard_uav_param_.origin_pos_offset[1];
                target_pos_.z() = onboard_received_.position.z - onboard_uav_param_.origin_pos_offset[2] + docking_param_.return_pos_offset[2];
                target_vel_ = Eigen::Vector3d::Zero();
                target_q_ = Eigen::Quaterniond::Identity();

                hover_flag_ = false;
                // is_first_run_ = true;
                docking_state_ = DockingStates::INIT;
                ROS_INFO("\033[32mDOCKING_RETRY: Switch to DOCKING IDLE\033[0m");
                // DEBUG 打印目标位姿
                //PrintUAVPosVel();
                break;
            }

            // 爬升到指定位置
            if (PlanTrajectory())
            {
                is_replan_ = false;
                // 打印目标位姿
                // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
            }
            break;
        }
    }
    return;
}

void OnboardUavFsm::FillLandingParams() {
    land_params_.is_landing = is_landing_;
    land_params_.uwb_dist = uwb_distance;
    land_params_.m_uav_pos.x() = m_uav_odom_pos_.x() - onboard_uav_param_.origin_pos_offset[0];
    land_params_.m_uav_pos.y() = m_uav_odom_pos_.y();
    land_params_.m_uav_pos.z() = m_uav_odom_pos_.z();

    if (is_landing_) {
        //调试使用
        // land_params_.land_x = 3.0;
        // land_params_.land_y = -4.0;
        // land_params_.land_z = 2.15;
        land_params_.land_x = landing_target_eskf_.pose.position.x;
        land_params_.land_y = landing_target_eskf_.pose.position.y;
        land_params_.land_z = landing_target_eskf_.pose.position.z;
    }
}

bool OnboardUavFsm::PlanTrajectory()
{
    int N = is_landing_ ? landing_minco_piece_ : normal_minco_piece_;

    if (!is_replan_ && !is_first_run_)
    {
        return true;
    }

    Eigen::MatrixXd start_state(3, 4);
    start_state.setZero();
    ros::Time now = ros::Time::now(); 
    double replan_time = (now - replan_start_time_).toSec();

    if (is_first_run_ || replan_time > poly_traj_.getTotalDuration())
    {
        start_state.col(0) = uav_odom_pos_;
        start_state.col(1) = uav_odom_vel_;
    }
    else
    {
        start_state.col(0) = poly_traj_.getPos(replan_time);
        start_state.col(1) = poly_traj_.getVel(replan_time);
        start_state.col(2) = poly_traj_.getAcc(replan_time);
        start_state.col(3) = poly_traj_.getJer(replan_time);
    }

    bool is_success = traj_opt_ptr_->generate_minco_traj(start_state, target_pos_, target_vel_, N, poly_traj_);
    if (is_success)
    {
        is_first_run_ = false;
        replan_start_time_ = now;
        //PubTrajectory函数将经过优化器优化后的轨迹poly_traj_发布
        PubTrajectory(replan_start_time_);

        traj_opt_ptr_->trans_bvp_traj(bvp_traj_);
        PubBvpTrajectory(now);
        vis_ptr_->visualize_traj(bvp_traj_, "bvp_trajectory"); 
        vis_ptr_->visualize_traj(poly_traj_, "onboard_uav_trajectory");
    }
    return is_success;
}

void OnboardUavFsm::PubTrajectory(const ros::Time &replan_start_time)
{
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 7;
    traj_msg.start_time = replan_start_time;
    traj_msg.traj_id = traj_id_++;

    Eigen::VectorXd durs = poly_traj_.getDurations();
    int piece_num = poly_traj_.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(8 * piece_num);
    traj_msg.coef_y.resize(8 * piece_num);
    traj_msg.coef_z.resize(8 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
        traj_msg.duration[i] = durs(i);
        CoefficientMat cMat = poly_traj_[i].getCoeffMat();
        int idx = 8 * i;
        for (int j = 0; j < 8; ++j)
        {
            traj_msg.coef_x[idx + j] = cMat(0, j);
            traj_msg.coef_y[idx + j] = cMat(1, j);
            traj_msg.coef_z[idx + j] = cMat(2, j);
        }
    }
    // 从 target_q_ 中获取航向角，ZYX顺序
    // Eigen::Matrix3d rotationMatrix = target_q_.toRotationMatrix();
    // Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(2, 1, 0);
    Eigen::Vector3d euler_angles = Quaterniond2EulerAngles(target_q_);
    traj_msg.yaw = euler_angles[0];

    trajectory_pub_.publish(traj_msg);
}

void OnboardUavFsm::PubBvpTrajectory(const ros::Time &start_time)
{
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 7;
    traj_msg.start_time = start_time;
    traj_msg.traj_id = traj_id_++;

    Eigen::VectorXd durs = bvp_traj_.getDurations();
    int piece_num = bvp_traj_.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(8 * piece_num);
    traj_msg.coef_y.resize(8 * piece_num);
    traj_msg.coef_z.resize(8 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
        traj_msg.duration[i] = durs(i);
        CoefficientMat cMat = bvp_traj_[i].getCoeffMat();
        int idx = 8 * i;
        for (int j = 0; j < 8; ++j)
        {
            traj_msg.coef_x[idx + j] = cMat(0, j);
            traj_msg.coef_y[idx + j] = cMat(1, j);
            traj_msg.coef_z[idx + j] = cMat(2, j);
        }
    }
    // 从 target_q_ 中获取航向角，ZYX顺序
    // Eigen::Matrix3d rotationMatrix = target_q_.toRotationMatrix();
    // Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(2, 1, 0);
    Eigen::Vector3d euler_angles = Quaterniond2EulerAngles(target_q_);
    traj_msg.yaw = euler_angles[0];

    bvp_traj_pub_.publish(traj_msg);
}

/**
 * @brief 发布悬停
 * 当需要打断轨迹继续执行时，发布悬停；否则等待轨迹执行完毕，px4ctrl会自动悬停
 */
void OnboardUavFsm::PubHoverPos()
{
    quadrotor_msgs::PolyTraj hover_traj_msg;
    hover_traj_msg.hover = true;
    hover_traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i)
    {
        hover_traj_msg.hover_p[i] = target_pos_[i];
    }
    hover_traj_msg.start_time = ros::Time::now();
    hover_traj_msg.traj_id = traj_id_++;
    trajectory_pub_.publish(hover_traj_msg);
}

void OnboardUavFsm::PubOnboardUavState()
{
    std_msgs::Float32 state_msg;

    state_msg.data = static_cast<float>(onboard_uav_state_);
    state_msg.data += 0.2 * static_cast<float>(docking_state_);
    state_msg.data += 0.04 * static_cast<float>(landing_state_);

    onboard_uav_state_pub_.publish(state_msg);
}

void OnboardUavFsm::Pub_px4_cmd(float target_x, float target_y, float target_z)
{
        px4_choose_msg.data = 1;
        px4_ctl_choose_.publish(px4_choose_msg);

        //px4 position control
        P_target.header.stamp = ros::Time::now();
        P_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        P_target.type_mask =			//使用位置控制
        //mavros_msgs::PositionTarget::IGNORE_PX |
        //mavros_msgs::PositionTarget::IGNORE_PY |
        //mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        //mavros_msgs::PositionTarget::IGNORE_YAW;
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        P_target.yaw = 0;
        P_target.position.x = target_x;
        P_target.position.y = target_y;
        P_target.position.z = target_z;

        position_ctl_pub_.publish(P_target);
}

void OnboardUavFsm::Pub_Guidance_State()
{
    guidance_state_.header.stamp = ros::Time::now();
    guidance_state_.uav_id = "Sub-UAV";
    guidance_state_.Guidance_mode = rg_stage_;
    guidance_state_.have_solution = rg_have_solution_;

    guidance_state_.geo_est_x = rg_last_mother_world_.x();
    guidance_state_.geo_est_y = rg_last_mother_world_.y();
    guidance_state_.geo_est_z = rg_last_mother_world_.z();
    guidance_state_.rg_sample_pos_x = rg_sample_pos_.x();
    guidance_state_.rg_sample_pos_y = rg_sample_pos_.y();
    guidance_state_.rg_sample_pos_z = rg_sample_pos_.z();

    guidance_state_.max_residual = rg_last_max_res_;
    guidance_state_.rms_residual = rg_last_rms_res_;

    guidance_state_.uwb_dis = uwb_distance;

    remote_ctrl_pub_.publish(guidance_state_);
}

void OnboardUavFsm::Pub_FSM_State()
{
    fsm_state_.header.stamp = ros::Time::now();
    fsm_state_.uav_id = 1;//Sub-UAV
    fsm_state_.fsm_state = onboard_uav_state_;
    fsm_state_.docking_state = docking_state_;
    fsm_state_.landing_state = static_cast<uint8_t>(landing_state_);

    fsm_state_.uav_id_s = "Sub-UAV";

    switch (onboard_uav_state_)
    {
        case OnboardUavStates::IDLE:
            fsm_state_.fsm_state_s = "IDLE";
            break;
        case OnboardUavStates::TAKEOFF:
            fsm_state_.fsm_state_s = "TAKEOFF";
            break;
        case OnboardUavStates::MISSION:
            fsm_state_.fsm_state_s = "MISSION";
            break;
        case OnboardUavStates::REMOTE_GUIDE:
            fsm_state_.fsm_state_s = "REMOTE_GUIDE";
            break;
        case OnboardUavStates::SEARCH:
            fsm_state_.fsm_state_s = "SEARCH";
            break;
        case OnboardUavStates::DOCKING:
            fsm_state_.fsm_state_s = "DOCKING";
            break;
        case OnboardUavStates::LAND:
            fsm_state_.fsm_state_s = "LAND";
            break;
        case OnboardUavStates::FAIL_SAFE:
            fsm_state_.fsm_state_s = "FAIL_SAFE";
            break;
        default:
            break;
    }

    if (onboard_uav_state_ == OnboardUavStates::DOCKING)
    {
        switch (docking_state_)
        {
            case DockingStates::INIT:
                fsm_state_.docking_state_s = "INIT";
                break;
            case DockingStates::RETURN:
                fsm_state_.docking_state_s = "RETURN";
                break;
            case DockingStates::LANDING:
                fsm_state_.docking_state_s = "LANDING";
                break;
            case DockingStates::COMPLETE:
                fsm_state_.docking_state_s = "COMPLETE";
                break;
            case DockingStates::RETRY:
                fsm_state_.docking_state_s = "RETRY";
                break;
            default:
                break;
        }
    }

    if (docking_state_ == DockingStates::LANDING)
    {
        switch (landing_state_)
        {
            case LandingStates::INIT:
                fsm_state_.landing_state_s = "INIT";
                break;
            case LandingStates::DESCEND_ABOVE_TARGET:
                fsm_state_.landing_state_s = "DESCEND_ABOVE_TARGET";
                break;
            case LandingStates::RE_CURRATE:
                fsm_state_.landing_state_s = "RE_CURRATE";
                break;           
            case LandingStates::FINAL_LANDING:
                fsm_state_.landing_state_s = "FINAL_LANDING";
                break;
            default:
                break;
        }
    }

    fsm_state_pub_.publish(fsm_state_);
}


/**
 * @brief 打印机载无人机当前状态
 */
void OnboardUavFsm::PrintOnboardUavState()
{

    switch (onboard_uav_state_)
    {
        case OnboardUavStates::IDLE:
            ROS_INFO("\033[32mOnboardUavState: IDLE\033[0m");
            break;
        case OnboardUavStates::TAKEOFF:
            ROS_INFO("\033[32mOnboardUavState: TAKEOFF\033[0m");
            break;
        case OnboardUavStates::MISSION:
            ROS_INFO("\033[32mOnboardUavState: MISSION\033[0m");
            break;
        case OnboardUavStates::DOCKING:
            ROS_INFO("\033[32mOnboardUavState: DOCKING\033[0m");
            break;
        case OnboardUavStates::LAND:
            ROS_INFO("\033[32mOnboardUavState: LAND\033[0m");
            break;
        case OnboardUavStates::FAIL_SAFE:
            ROS_INFO("\033[32mOnboardUavState: FAIL_SAFE\033[0m");
            break;
        default:
            break;
    }

    if (onboard_uav_state_ == OnboardUavStates::DOCKING)
    {
        switch (docking_state_)
        {
            case DockingStates::INIT:
                ROS_INFO("\033[32mDockingState: INIT\033[0m");
                break;
            case DockingStates::RETURN:
                ROS_INFO("\033[32mDockingState: RETURN\033[0m");
                break;
            case DockingStates::LANDING:
                ROS_INFO("\033[32mDockingState: LANDING\033[0m");
                break;
            case DockingStates::COMPLETE:
                ROS_INFO("\033[32mDockingState: COMPLETE\033[0m");
                break;
            case DockingStates::RETRY:
                ROS_INFO("\033[32mDockingState: RETRY\033[0m");
                break;
            default:
                break;
        }
    }

    if (docking_state_ == DockingStates::LANDING)
    {
        switch (landing_state_)
        {
            case LandingStates::INIT:
                ROS_INFO("\033[32mLandingState: INIT\033[0m");
                break;
            case LandingStates::DESCEND_ABOVE_TARGET:
                ROS_INFO("\033[32mLandingState: DESCEND_ABOVE_TARGET\033[0m");
                break;
            case LandingStates::FINAL_LANDING:
                ROS_INFO("\033[32mLandingState: FINAL_LANDING\033[0m");
                break;
            default:
                break;
        }
    }
}

/**
 * @brief 打印无人机当前位置、速度、目标位置、速度
 */
void OnboardUavFsm::PrintUAVPosVel()
{
    if (!OdomIsReceived(ros::Time::now()))
    {
        return;
    }

    // 打印无人机当前位置、速度
    std::cout << std::fixed << std::setprecision(2) << "[current] pos: " << uav_odom_pos_.transpose() << ";  ";
    std::cout << std::fixed << std::setprecision(2) << "vel: " << uav_odom_vel_.transpose() << std::endl;

    // 打印目标位置、速度
    if (hover_flag_)
    {
        std::cout << std::fixed << std::setprecision(2) << "[target]  pos: " << target_pos_.transpose() << std::endl;
    }
    else
    {
        std::cout << std::fixed << std::setprecision(2) << "[target]  pos: " << target_pos_.transpose() << ";  ";
        std::cout << std::fixed << std::setprecision(2) << "vel: " << target_vel_.transpose() << std::endl;
    }
}

