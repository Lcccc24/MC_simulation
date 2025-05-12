#include "planner/onboard_uav_fsm.hpp"

// Header header
// uint8 uav_id # 本机ID
// uint8 target_uav_id # 目标无人机ID
// uint8 flight_command # 母机飞行指令：0-无指令 1-起飞 2-任务 3-对接 4-降落
// uint8 flight_status # 子机飞行状态：10-起飞完成 20-任务完成 40-降落完成 40-返航完成 50-精准降落完成
// geometry_msgs/Point position # 位置： 任务点 返航点

// # 母机飞行指令
// uint8 IDLE = 0 # 待机
// uint8 TAKEOFF =1 # 起飞
// uint8 MISSION = 2 # 执行任务
// uint8 DOCKING = 3 # 对接
// uint8 LAND = 4 # 降落
// uint8 ALLOW_RETURN = 5 # 允许返航
// uint8 ALLOW_PRECISION_LANDING = 6 # 允许精准降落

// # 子机飞行状态
// uint8 TAKEOFF_COMPLETE = 10 # 起飞完成
// uint8 MISSION_COMPLETE = 20 # 任务完成
// uint8 LAND_COMPLETE = 30 # 降落完成
// uint8 RETURN_COMPLETE = 40 # 返航完成
// uint8 PRECISION_LANDING_COMPLETE = 50 # 精准降落完成

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
    nh.param("docking/allowed_landing_time_s", docking_param_.allowed_landing_time_s, 300.0);
    nh.param("docking/allowed_final_landing_time_s", docking_param_.allowed_final_landing_time_s, 10.0);
    nh.param("docking/allowed_retry_hover_time_s", docking_param_.allowed_retry_hover_time_s, 10.0);
    nh.param("docking/retry_climb_height", docking_param_.retry_climb_height, 2.0);
    nh.param("docking/allowed_retry_num", docking_param_.allowed_retry_num, 10);

    nh.param("msg_timeout/odom", msg_timeout_.odom, 0.5);
    nh.param("msg_timeout/onboard", msg_timeout_.onboard, 0.5);
    nh.param("msg_timeout/landing_target", msg_timeout_.landing_target, 0.1);

    uav_local_pose_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/pose", 1, &OnboardUavFsm::UavLocalPoseCallback, this);
    uav_local_vel_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/velocity_local", 1, &OnboardUavFsm::UavLocalVelCallback, this);

    uav_state_sub_ = nh.subscribe("/Sub_UAV/mavros/state", 1, &OnboardUavFsm::UavStateCallback, this);
    uav_odom_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/odom", 1, &OnboardUavFsm::UavOdomCallback, this);
    std::string onboard_msg_sub_name = "/uav" + std::to_string(onboard_uav_param_.uav_id) + "/onboard_msg";
    onboard_msg_sub_ = nh.subscribe(onboard_msg_sub_name, 1, &OnboardUavFsm::OnboardMsgCallback, this);
    // std::string landing_target_pose_topic_name = std::to_string(onboard_uav_param_.uav_id) + "/landing_target_pose";
    landing_target_pose_sub_ = nh.subscribe("/landing_target_pose/ESKF", 1, &OnboardUavFsm::LandingTargetPoseCallback, this);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/heartbeat", 1); 
    takeoff_land_cmd_pub_ = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    trajectory_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("/trajectory", 1);
    std::string onboard_msg_pub_name = "/uav" + std::to_string(onboard_uav_param_.target_uav_id) + "/onboard_msg";
    onboard_msg_pub_ = nh.advertise<quadrotor_msgs::Onboard>(onboard_msg_pub_name, 1);
    arm_disarm_client_ = nh.serviceClient<mavros_msgs::CommandLong>("/Sub_UAV/mavros/cmd/command");
    onboard_uav_state_pub_ = nh.advertise<std_msgs::Float32>("/onboard_uav_state", 1);

    //lc add
    px4_ctl_choose_ = nh.advertise<std_msgs::Int32>("/px4_ctl_choose", 10);
    position_ctl_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/Sub_UAV/mavros/setpoint_raw/local", 10);

    mother_move_pub_ = nh.advertise<std_msgs::Int32>("/mother_move/cmd",1);

    //uwb_distance_sub_  = nh.subscribe("nlink_linktrack_nodeframe2", 1000, &OnboardUavFsm::Uwb_distance_callback,this);
    uwb_distance_sub_  = nh.subscribe("/fake_uwb_distance", 10, &OnboardUavFsm::Uwb_distance_callback,this);
    
    //远程引导标志位pub与进入降落标志位pub 
    remote_ctrl_pub_ = nh.advertise<quadrotor_msgs::GuidanceState>("/remote_ctrl/state", 1);
    fsm_state_pub_ = nh.advertise<quadrotor_msgs::FsmState>("/fsm_state", 1);

    // 创建路径规划器实例
    traj_opt_ptr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    // 创建可视化实例
    vis_ptr_ = std::make_shared<vis_utils::VisUtils>(nh);

    // 初始化线程
    uav_disarm_thread_ = std::thread(&OnboardUavFsm::UavDisarm, this);



    fsm_hz_ = 50;
    replan_hz_ = 2;
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
    docking_state_ = DockingStates::INIT;https://www.chatopens.com/
    landing_state_ = LandingStates::INIT;
    retry_state_ = RetryStates::INIT;

    traj_id_ = 0;
    is_landing_target_pose_updated_ = false;
    hover_flag_ = false;
    is_docking_retry_ = false;
    // 用默认值初始化
    // uav_local_pose_ = {};
    // uav_local_vel_ = {};
    uav_odom_ = {};
    onboard_received_ = {};
    landing_target_pose_ = {};
    target_pos_ = Eigen::Vector3d::Zero();
    target_vel_ = Eigen::Vector3d::Zero();
    target_q_ = Eigen::Quaterniond::Identity();

    perform_uav_disarm_ = false;
    is_replan_ = true;
    is_first_run_ = true;
    replan_start_time_ = ros::Time(0.0);

    //lc add
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


//lc add uwb距离数据订阅回调函数
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

/**
 * @brief 无人机里程计订阅回调函数
 * @param msg 无人机里程计消息
 */
void OnboardUavFsm::UavOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // while (odom_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    uav_odom_ = *msg;
    // odom_lock_.clear(std::memory_order_release);
}

/**
 * @brief 母机指令订阅回调函数
 * @param msg 母机指令消息
 */
void OnboardUavFsm::OnboardMsgCallback(const quadrotor_msgs::Onboard::ConstPtr &msg)
{
    // TODO 筛选无人机 ID
    // onboard_msg_lock_
    // while (onboard_msg_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    onboard_received_ = *msg;
    //ROS_INFO("onboard_received_ : %f,%f,%f",onboard_received_.position.x,onboard_received_.position.y,onboard_received_.position.z);
    
    // onboard_msg_lock_.clear(std::memory_order_release);
}

/**
 * @brief 降落目标位置订阅回调函数
 * @param msg 降落目标位置消息
 */
void OnboardUavFsm::LandingTargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // while (landing_target_pose_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    landing_target_pose_ = *msg;
    is_landing_target_pose_updated_ = true;
    // landing_target_pose_lock_.clear(std::memory_order_release);
}

/**
 * @brief 判断里程计是否接收到
 * @param now_time 时间
 * @return bool 是否接收到
 */
bool OnboardUavFsm::OdomIsReceived(const ros::Time &now_time)
{
    // while (odom_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    bool is_received = now_time.toSec() - uav_local_pose_.header.stamp.toSec() < msg_timeout_.odom;
    // odom_lock_.clear(std::memory_order_release);
    return is_received;
}

/**
 * @brief 判断母机指令是否接收到
 * @param now_time 时间
 * @return bool 是否接收到
 */
bool OnboardUavFsm::OnboardMsgIsReceived(const ros::Time &now_time)
{
    // while (onboard_msg_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    bool is_received = now_time.toSec() - onboard_received_.header.stamp.toSec() < msg_timeout_.onboard;
    // onboard_msg_lock_.clear(std::memory_order_release);
    return is_received;
}

/**
 * @brief 判断降落目标位置是否接收到
 * @param now_time 时间
 * @return bool 是否接收到
 */
bool OnboardUavFsm::LandingTargetPoseIsReceived(const ros::Time &now_time)
{
    // TODO timeout 需要小于两倍的话题发布周期？这样会不会太短？
    // while (landing_target_pose_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    // 打印时间戳
    //ROS_WARN("landing_target_pose_ timeout: %f, %f", now_time.toSec(), landing_target_pose_.header.stamp.toSec());
    bool is_received = now_time.toSec() - landing_target_pose_.header.stamp.toSec() < msg_timeout_.landing_target;
    // landing_target_pose_lock_.clear(std::memory_order_release);
    return is_received;
}

/**
 * @brief 判断母机指令是否为某个模式
 * @param flight_command 模式
 * @return bool 是否为某个模式
 */
bool OnboardUavFsm::IsOnboardCommand(const int &flight_command)
{
    // while (onboard_msg_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    bool is_mode = (onboard_received_.flight_command == flight_command);
    // onboard_msg_lock_.clear(std::memory_order_release);
    return is_mode;
}

/**
 * @brief 判断母机指令是否为某个状态
 * @param flight_status 状态
 * @return bool 是否为某个状态
 */
// bool OnboardUavFsm::IsOnboardStatus(const int &flight_status)
// {
//     while (onboard_msg_lock_.test_and_set(std::memory_order_acquire))
//         ;
//     bool is_status = (onboard_received_.flight_status == flight_status);
//     onboard_msg_lock_.clear(std::memory_order_release);
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
        // while (odom_lock_.test_and_set(std::memory_order_acquire))
        //     ;
        uav_odom_pos_ << uav_local_pose_.pose.position.x, uav_local_pose_.pose.position.y, uav_local_pose_.pose.position.z;
        uav_odom_vel_ << uav_local_vel_.twist.linear.x, uav_local_vel_.twist.linear.y, uav_local_vel_.twist.linear.z;
        // uav_odom_orient_.x() = uav_local_pose_.pose.orientation.x;
        // uav_odom_orient_.y() = uav_local_pose_.pose.orientation.y;
        // uav_odom_orient_.z() = uav_local_pose_.pose.orientation.z;
        // uav_odom_orient_.w() = uav_local_pose_.pose.orientation.w;
        // odom_lock_.clear(std::memory_order_release);
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

    switch (onboard_uav_state_)
    {
    case OnboardUavStates::IDLE:
    {
        // 当接收到母机的起飞指令时，进入起飞状态
        if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::TAKEOFF))
        // if ((OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::TAKEOFF)) || uav_state_.armed)
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
            onboard_published_.flight_command = quadrotor_msgs::Onboard::TAKEOFF;
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

        // 出现故障时，进入故障保护状态
        // 此处状态切换有点乱 后续需要修改
        // 当接收到母机的对接指令时，进入对接状态
        if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::DOCKING))
        {
            hover_flag_ = false;
            // is_first_run_ = true;
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

        RunMissionMode();
        //Remote_Guidance();
        break;
    }

    case OnboardUavStates::REMOTE_GUIDE:
    {
        // 当接收到母机的对接指令时，进入对接状态
        if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::DOCKING))
        {
            hover_flag_ = false;
            // is_first_run_ = true;
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

        Remote_Guidance();
        break;
    }

    case OnboardUavStates::DOCKING:
    {
        // TODO disarmed后需要发布状态给母机
        // if (uav_state_.mode != "OFFBOARD")
        // {
        //     break;
        // }

        // 出现故障时，进入故障保护状态

        // 当接收到母机的任务指令时，进入任务状态
        if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::MISSION))
        {
            traj_opt_ptr_->setLandingParams(false);
            is_first_run_ = true;
            onboard_uav_state_ = OnboardUavStates::MISSION;
            ROS_INFO("\033[32mDOCKING: Switch to MISSION\033[0m");
            break;
        }
        // 当接收到母机的降落指令时，进入降落状态
        else if (OnboardMsgIsReceived(now_time) && IsOnboardCommand(quadrotor_msgs::Onboard::LAND))
        {
            traj_opt_ptr_->setLandingParams(false);
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
            onboard_published_.flight_command = quadrotor_msgs::Onboard::LAND;
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
    static int count;
    if(rg_flag == 0)
    {
        // 执行任务
        ROS_INFO("RUN MISSION POINT");
        if (is_first_run_)
        {
            // 设置航点
            target_pos_.x() = onboard_received_.position.x - onboard_uav_param_.origin_pos_offset[0];
            target_pos_.y() = onboard_received_.position.y - onboard_uav_param_.origin_pos_offset[1];
            target_pos_.z() = onboard_received_.position.z - onboard_uav_param_.origin_pos_offset[2];
            target_vel_ = Eigen::Vector3d::Zero();
            target_q_ = Eigen::Quaterniond::Identity();
            hover_flag_ = false;
            ROS_INFO("target_pos_ : %f,%f,%f",target_pos_.x(),target_pos_.y(),target_pos_.z());
            // DEBUG 打印目标位姿
        }

        // 到达指定点后，发送 Onboard::MISSION_COMPLETE，等待下一步指令
        if ((uav_odom_pos_ - target_pos_).norm() < 0.3)
        {
            count ++;
            if(count > 0 && count < 5){
                std_msgs::Int32 mother_move_msg;
                mother_move_msg.data = 1;
                mother_move_pub_.publish(mother_move_msg);
            }

            if(count > 350){
                count = 0;
                onboard_published_.flight_command = quadrotor_msgs::Onboard::MISSION;
                //onboard_published_.flight_status = quadrotor_msgs::Onboard::MISSION_COMPLETE;
                PubOnboardMsg();
                rg_flag = 1;
                // 发布悬停
                // if (!hover_flag_)
                // {
                //     // target_pos_ = uav_odom_pos_;
                //     hover_flag_ = true;
                //     // PubHoverPos();
                //     ROS_INFO("MISSION COMPLETE: Hover"); // TODO 有bug，会一直打印
                //执行完任务后进入远程引导
                onboard_uav_state_= OnboardUavStates::REMOTE_GUIDE;
                // }
                return;
            }
        }

        // 调用规划器，规划任务轨迹
        if (!hover_flag_ && PlanTrajectory())
        {
            is_replan_ = false;
        }

    }

    else
        return;
}

//lc add
/**
 * @brief 远程引导模式
 */
void OnboardUavFsm::Remote_Guidance()
{   

        //PUB FLAG
    Pub_Guidance_State();

    static int i = 0;
    static int hover_times = 0;
    static int k = 0;
    int mean_time = 50;
    ROS_INFO("RG_FLAG:%d",rg_flag);
    if(rg_flag == 1)
    {
        // 确保索引 i 在 p_set 的范围内
        if (i >= p_set.size()) 
        {
            ROS_WARN("Index i (%d) is out of bounds for p_set size (%ld)", i, p_set.size());
            return;
        }

        // 设置航点
        target_pos_.x() = onboard_received_.position.x - onboard_uav_param_.origin_pos_offset[0] + p_set[i].x();
        target_pos_.y() = onboard_received_.position.y - onboard_uav_param_.origin_pos_offset[1] + p_set[i].y();
        target_pos_.z() = onboard_received_.position.z - onboard_uav_param_.origin_pos_offset[2] + p_set[i].z();
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_ = Eigen::Quaterniond::Identity();
        hover_flag_ = false;
        //ROS_INFO("target_pos_%d : %f,%f,%f",i+1,target_pos_.x(),target_pos_.y(),target_pos_.z());

        if ((uav_odom_pos_ - target_pos_).norm() < 0.3)
        {
            ROS_INFO("PREVIEW POINT %d",i+1);
            hover_times ++;
            if(hover_times > 15)
            {
                if(k < mean_time)
                {
                    k++;
                    pre_uwb_d[i] += uwb_distance;
                    pre_vio_p[i].x() += uav_odom_pos_.x() - (onboard_received_.position.x - onboard_uav_param_.origin_pos_offset[0]);
                    pre_vio_p[i].y() += uav_odom_pos_.y() - (onboard_received_.position.y - onboard_uav_param_.origin_pos_offset[1]);
                    pre_vio_p[i].z() += uav_odom_pos_.z() - (onboard_received_.position.z - onboard_uav_param_.origin_pos_offset[2]);
                    //ROS_INFO("PRE_UWB_i%d : %f",i+1,pre_uwb_d[i]/k);
                    //ROS_INFO("PRE_VIO_i%d : %f,%f,%f",i+1,pre_vio_p[i].x()/k,pre_vio_p[i].y()/k,pre_vio_p[i].z()/k);
                    // ros::spinOnce();
                    // ros::Duration(0.05).sleep(); // 添加延迟模拟合理采样间隔
                }

                else{
                    pre_uwb_d[i] = pre_uwb_d[i] / mean_time;
                    pre_vio_p[i] = pre_vio_p[i] / mean_time;
                    ROS_INFO("PRE_UWB_%d : %f",i+1,pre_uwb_d[i]);
                    ROS_INFO("PRE_VIO_%d : %f,%f,%f",i+1,pre_vio_p[i].x(),pre_vio_p[i].y(),pre_vio_p[i].z());
                    i++;
                    k = 0;
                    hover_times = 0;
                }
            }

            if(i == 4)
                rg_flag = 2;
        }

        //规划器--px4ctl接口
        if(PlanTrajectory());
        {
            is_replan_ = false;
            // 打印目标位姿
            // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
        }

    }


    if(rg_flag == 2)
    {

        geometric_estimate();
        target_pos_ = geo_est_c2d + dock_r2m;
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_ = Eigen::Quaterniond::Identity();

        if((uav_odom_pos_ - target_pos_).norm() > d_change)
        {
            ROS_INFO("geometric method guidance");
            ROS_INFO("uav_odom_pos_ : %f,%f,%f",uav_odom_pos_.x(),uav_odom_pos_.y(),uav_odom_pos_.z());
        }


        else
            rg_flag = 3;

        if(PlanTrajectory());
        {
            is_replan_ = false;
            // 打印目标位姿
            // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
        }
        
    }

    if(rg_flag == 3)
    {
        iterate_estimate();
        target_pos_ = ite_est_p;
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_ = Eigen::Quaterniond::Identity();
        //ROS_INFO("iterate method guidance");
        //ROS_INFO("target_pos_ : %f,%f,%f",target_pos_.x(),target_pos_.y(),target_pos_.z());
    

        if((ite_c2d_q - dock_r2m).norm() < d_stopite && ite_k > 50)
        //if((uav_odom_pos_ - geo_est_c2d - dock_r2m).norm() < d_stopite && ite_k > 10)
        {
            onboard_published_.flight_command = quadrotor_msgs::Onboard::MISSION;
            onboard_published_.flight_status = quadrotor_msgs::Onboard::MISSION_COMPLETE;
            PubOnboardMsg();
            // PlanTrajectory();
            // px4_choose_msg.data = 0;
            // px4_ctl_choose_.publish(px4_choose_msg);
            ROS_INFO("REMOTE GUIDANCE COMPLETE"); 
            return;
        }
        
        //px4 position control
        //Pub_px4_cmd(ite_est_p.x(), ite_est_p.y(), ite_est_p.z());
        if(PlanTrajectory());
        {
            is_replan_ = false;
            // 打印目标位姿
            // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
        }

    }



    else
        return;

}


void OnboardUavFsm::geometric_estimate()
{
    Eigen::Matrix3d A; 
    Eigen::Vector3d b;

    static Eigen::Vector3d act_c2d = {onboard_received_.position.x,onboard_received_.position.y,onboard_received_.position.z};
    static Eigen::Vector3d offset = {onboard_uav_param_.origin_pos_offset[0],onboard_uav_param_.origin_pos_offset[1],onboard_uav_param_.origin_pos_offset[2]};

    A << 2*(pre_vio_p[1].x() - pre_vio_p[0].x()), 2*(pre_vio_p[1].y() - pre_vio_p[0].y()), 2*(pre_vio_p[1].z() - pre_vio_p[0].z()),
    2*(pre_vio_p[2].x() - pre_vio_p[0].x()), 2*(pre_vio_p[2].y() - pre_vio_p[0].y()), 2*(pre_vio_p[2].z() - pre_vio_p[0].z()),
    2*(pre_vio_p[3].x() - pre_vio_p[0].x()), 2*(pre_vio_p[3].y() - pre_vio_p[0].y()), 2*(pre_vio_p[3].z() - pre_vio_p[0].z());

    b << pre_uwb_d[0]*pre_uwb_d[0] - pre_uwb_d[1]*pre_uwb_d[1] + pre_vio_p[1].squaredNorm() - pre_vio_p[0].squaredNorm(),
    pre_uwb_d[0]*pre_uwb_d[0] - pre_uwb_d[2]*pre_uwb_d[2] + pre_vio_p[2].squaredNorm() - pre_vio_p[0].squaredNorm(),
    pre_uwb_d[0]*pre_uwb_d[0] - pre_uwb_d[3]*pre_uwb_d[3] + pre_vio_p[3].squaredNorm() - pre_vio_p[0].squaredNorm();

    //求解矩阵得到几何法的子机相对于对接点的位置
    //局部坐标系，以收到的任务目标点为原点，解算出的为母机在局部坐标系下的位置
    geo_est_c2d = A.inverse() * b;
    ROS_INFO("geo_est_c2d: [%f, %f, %f]", geo_est_c2d.x(), geo_est_c2d.y(), geo_est_c2d.z());

    //子机惯性坐标系即全局坐标系下的位置
    geo_est_c2d = (geo_est_c2d + act_c2d);
    ROS_INFO("geo_est_c2d: [%f, %f, %f]", geo_est_c2d.x(), geo_est_c2d.y(), geo_est_c2d.z());

    //仿真中初始位置字母机不一致，存在位置偏移量
    geo_est_c2d = geo_est_c2d - offset;
    ROS_INFO("geo_est_c2d: [%f, %f, %f]", geo_est_c2d.x(), geo_est_c2d.y(), geo_est_c2d.z());
}


void OnboardUavFsm::iterate_estimate()
{
    //子机相对于对接点的位置
    static Eigen::Vector3d vio_pk, vio_pk0, Delta_pk, ite_vel, sigma;
    static Eigen::Vector2d rho;
    static float uwb_intermediate, error_k;
    static float a = 72;
    static float theta = 0;
    static float b1_val = 1/2;
    static float b2_val = sqrt(3)/2;
    static float gamma_val = 10;
    static float alpha_val = 3;
    static float beta_value = 8;
    static double vel_max = 10.0f;
    static double uwb_dk,uwb_dk0;
    static float iterate_time = 0.1f;
    static Eigen::Matrix2d D;
    D << cos(2*M_PI/a), -sin(2*M_PI/a),
    sin(2*M_PI/a), cos(2*M_PI/a);


    if(ite_k == 0)
    {   
        vio_pk0 = uav_odom_pos_;
        uwb_dk0 = uwb_distance;
        rho << cos(theta), sin(theta);
        sigma << b1_val*rho.x(), b1_val*rho.y(), b2_val*(4*pow(rho.x(),3) - 3*rho.x());
        //ite_c2d_q = {1,1,1};
        ite_c2d_q = uav_odom_pos_ - geo_est_c2d;
        ite_vel = beta_value*(dock_r2m - ite_c2d_q) + alpha_val*sigma;
        if(ite_vel.norm() > vel_max)
            ite_vel = ite_vel / ite_vel.norm() * vel_max;
        ite_est_p = ite_vel * iterate_time;
        ite_est_p = vio_pk0 + ite_est_p;
        ite_k++;

    }

    else
    {
        vio_pk = uav_odom_pos_;
        uwb_dk = uwb_distance; 
        Delta_pk = vio_pk - vio_pk0;
        rho = D*rho;
        //sigma << b1_val*(4*pow(rho.x(),3) - 3*rho.x()), b1_val*(3*rho.y() - 4*pow(rho.y(),3)), b2_val*rho.x(); 
        sigma << b1_val*rho.x(), b1_val*rho.y(), b2_val*(4*pow(rho.x(),3) - 3*rho.x());
        uwb_intermediate = (pow(uwb_dk,2) - pow(uwb_dk0,2) - pow(Delta_pk.norm(),2)) / 2;
        error_k = uwb_intermediate - Delta_pk.transpose()*ite_c2d_q;
        ite_c2d_q = (ite_c2d_q + Delta_pk + gamma_val*Delta_pk*error_k);
        ite_c2d_q = ite_c2d_q * uwb_dk / std::max(ite_c2d_q.norm(),uwb_dk);
        ite_vel = beta_value*(dock_r2m - ite_c2d_q) + alpha_val*sigma;
        ite_vel = ite_vel * vel_max / std::max(ite_vel.norm(),vel_max);
        if(ite_vel.norm() > vel_max)
            ite_vel = ite_vel / ite_vel.norm() * vel_max;
        ite_est_p = ite_vel * iterate_time;
        ROS_INFO("ite_pos: [%f, %f, %f]", ite_est_p.x(), ite_est_p.y(), ite_est_p.z());
        ite_est_p = vio_pk + ite_est_p;

        //update parameter
        vio_pk0 = vio_pk;
        uwb_dk0 = uwb_dk;
        ite_k++;
    }

    ROS_INFO("k:bias: %d,%f,%f,%f",ite_k,ite_c2d_q.x(),ite_c2d_q.y(),ite_c2d_q.z());
    //ROS_INFO("k:bias: %d,%f,%f,%f",ite_k,uav_odom_pos_.x() - geo_est_c2d.x()-dock_r2m.x(),uav_odom_pos_.y() - geo_est_c2d.y()-dock_r2m.y(),uav_odom_pos_.z() - geo_est_c2d.z()-dock_r2m.z());
    ROS_INFO("ite_vel: [%f, %f, %f]", ite_vel.x(), ite_vel.y(), ite_vel.z());
    ROS_INFO("now_pos: [%f, %f, %f]", vio_pk.x(), vio_pk.y(), vio_pk.z());
    ROS_INFO("target_pos: [%f, %f, %f]", ite_est_p.x(), ite_est_p.y(), ite_est_p.z());

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
        // 当标签不可见，且不是 FINAL_LANDING 时，进入重试状态
        // 当降落超时时，进入故障保护状态
        traj_opt_ptr_->setLandingParams(true);
        RunDockingLanding();
        break;
    }

    case DockingStates::COMPLETE:
    {
        // 关闭电机
        RunDockingComplete();
        break;
    }
    case DockingStates::RETRY:
    {
        // 当处于 HOVER_SEARCH 状态，且标签可见时，进入降落状态
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
        docking_state_ = DockingStates::RETURN;
        ROS_INFO("\033[32mDOCKING_IDLE: Switch to DOCKING RETURN\033[0m");
    }
    else
    {
        onboard_published_.flight_command = quadrotor_msgs::Onboard::DOCKING;
        onboard_published_.flight_status = quadrotor_msgs::Onboard::MISSION_COMPLETE;
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
        // while (onboard_msg_lock_.test_and_set(std::memory_order_acquire))
        //     ;

        //lc change
        target_pos_ = ite_est_p;

        // onboard_msg_lock_.clear(std::memory_order_release);
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_ = Eigen::Quaterniond::Identity();
        //ROS_INFO("111");
        // DEBUG 打印目标位姿
        //PrintUAVPosVel();
    }

    // 当接收到母机的降落指令，且标签可见时，进入降落状态，
    // 标签不可见时，进一步到达返航位置，仍然不可见时，悬停，此时还会发送 Onboard::RETURN_COMPLETE
    // TODO 让母机在重试时也发送降落指令
    if ((OnboardMsgIsReceived(ros::Time::now()) && IsOnboardCommand(quadrotor_msgs::Onboard::ALLOW_PRECISION_LANDING)) || is_docking_retry_)
    // if (IsOnboardCommand(quadrotor_msgs::Onboard::ALLOW_PRECISION_LANDING))
    {

        if (LandingTargetPoseIsReceived(ros::Time::now()))
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
            //lc change
            target_pos_ = ite_est_p;
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
    if ((uav_odom_pos_ - target_pos_).norm() < 0.4)
    {
        onboard_published_.flight_command = quadrotor_msgs::Onboard::DOCKING;
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
    // 当标签不可见，且不是 FINAL_LANDING 时，进入重试状态
    // TODO FINAL_LANDING需要限制时间，超时进入重试状态
    //if (!LandingTargetPoseIsReceived(ros::Time::now()) && landing_state_ != LandingStates::FINAL_LANDING)
    if (!LandingTargetPoseIsReceived(ros::Time::now()) )
    {
        traj_opt_ptr_->setLandingParams(false);
        retry_state_ = RetryStates::INIT;
        docking_state_ = DockingStates::RETRY;
        ROS_WARN("DOCKING_LANDING: Landing target lost! Switch to RETRY");
        return;
    }

    // 计算无人机与标签的水平距离
    static Eigen::Vector3d relative_pos = Eigen::Vector3d::Zero();
    static double horizontal_distance = 0.0;
    // while (landing_target_pose_lock_.test_and_set(std::memory_order_acquire))
    //     ;
    relative_pos.x() = landing_target_pose_.pose.position.x - uav_odom_pos_.x();
    relative_pos.y() = landing_target_pose_.pose.position.y - uav_odom_pos_.y();
    relative_pos.z() = landing_target_pose_.pose.position.z - uav_odom_pos_.z();
    
    //ROS_INFO("relative_pos:%f,%f,%f",relative_pos.x(),relative_pos.y(),relative_pos.z());
    // landing_target_pose_lock_.clear(std::memory_order_release);
    // horizontal_distance = std::sqrt(relative_pos.x() * relative_pos.x() + relative_pos.y() * relative_pos.y());
    horizontal_distance = relative_pos.head(2).norm();

    switch (landing_state_)
    {
    case LandingStates::INIT:
    {
        docking_landing_start_time_ = ros::Time::now();
        is_first_run_ = true;
        // landing_state_ = LandingStates::HORIZONTAL_APPROACH;
        // ROS_INFO("\033[32mDOCKING_INIT: Switch to HORIZONTAL_APPROACH\033[0m");

        landing_state_ = LandingStates::DESCEND_ABOVE_TARGET;
        ROS_INFO("\033[32mDOCKING_INIT: Switch to DESCEND_ABOVE_TARGET\033[0m");
        
        break;
    }

    case LandingStates::DESCEND_ABOVE_TARGET:
    {
        // // 当无人机与标签的水平距离大于水平距离容差时，重新进入水平接近状态
        // if (horizontal_distance > 1.2 * docking_param_.horizontal_distance_tolerance)
        // {
        //     is_first_run_ = true;
        //     landing_state_ = LandingStates::HORIZONTAL_APPROACH;
        //     ROS_INFO("\033[32mDOCKING_DESCEND_ABOVE_TARGET: Switch to HORIZONTAL_APPROACH\033[0m");
        //     break;
        // }
        // 当无人机与标签的垂直距离小于最终下降高度，且水平距离小于允许降落误差时，进入最终降落状态

        static int landing_descend_complete_count = 0;

        if (std::fabs(relative_pos.z()) < docking_param_.descend_ver_bias && horizontal_distance < docking_param_.descend_ver_bias)
        {
            landing_descend_complete_count++;
            // is_first_run_ = true; // 飞行中不需要重新规划
            if(landing_descend_complete_count > 5)
            {
                final_landing_start_time_ = ros::Time::now();
                landing_state_ = LandingStates::RE_CURRATE;
                ROS_INFO("\033[32mDOCKING_DESCEND_ABOVE_TARGET: Switch to RE_CURRATE\033[0m");
                break;
            }
        }

        else
        {
            landing_descend_complete_count = 0;
        }

        target_pos_.x() = landing_target_pose_.pose.position.x;
        target_pos_.y() = landing_target_pose_.pose.position.y;
        target_pos_.z() = landing_target_pose_.pose.position.z + docking_param_.descend_ver_bias;
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_.x() = landing_target_pose_.pose.orientation.x;
        target_q_.y() = landing_target_pose_.pose.orientation.y;
        target_q_.z() = landing_target_pose_.pose.orientation.z;
        target_q_.w() = landing_target_pose_.pose.orientation.w;

        // 调用规划器，规划在目标上方下降轨迹
        ROS_INFO("des:TARGET_POS:%.2f,%.2f,%.2f\n",target_pos_.x(),target_pos_.y(),target_pos_.z());
        if(PlanTrajectory());
        {
            is_replan_ = false;
            // 打印目标位姿
            // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
        }

        //lc change
        //Pub_px4_cmd(target_pos_.x(),target_pos_.y(),target_pos_.z());
        

        break;
    }

    case LandingStates::RE_CURRATE:
    {
        static int landing_rec_count = 0;

        if (std::fabs(relative_pos.z()) < docking_param_.rec_ver_bias && horizontal_distance < docking_param_.rec_hor_bias)
        {
            landing_rec_count++;
            // is_first_run_ = true; // 飞行中不需要重新规划
            if(landing_rec_count > 5)
            {
                final_landing_start_time_ = ros::Time::now();
                landing_state_ = LandingStates::FINAL_LANDING;
                ROS_INFO("\033[32mDOCKING_RE_CURRATE: Switch to FINAL_LANDING\033[0m");
                break;
            }
        }

        else
        {
            landing_rec_count = 0;
        }

        target_pos_.x() = landing_target_pose_.pose.position.x;
        target_pos_.y() = landing_target_pose_.pose.position.y;
        target_pos_.z() = landing_target_pose_.pose.position.z + docking_param_.rec_hor_bias;
        target_vel_ = Eigen::Vector3d::Zero();
        target_q_.x() = landing_target_pose_.pose.orientation.x;
        target_q_.y() = landing_target_pose_.pose.orientation.y;
        target_q_.z() = landing_target_pose_.pose.orientation.z;
        target_q_.w() = landing_target_pose_.pose.orientation.w;

        // 调用规划器，规划在目标上方下降轨迹
        ROS_INFO("re:TARGET_POS:%.2f,%.2f,%.2f\n",target_pos_.x(),target_pos_.y(),target_pos_.z());
        if (PlanTrajectory())
        {
            is_replan_ = false;
            // 打印目标位姿
            // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
        }

        //lc change
        //Pub_px4_cmd(target_pos_.x(),target_pos_.y(),target_pos_.z());
        

        break;
    }


    case LandingStates::FINAL_LANDING:
    {

        // 当无人机与标签的垂直距离小于降落完成高度，且水平距离小于允许降落误差时，进入降落完成状态
        static int landing_complete_count = 0;
        static int landing_horizon_error = 0;
        if (std::fabs(relative_pos.z()) < docking_param_.final_ver_bias && horizontal_distance < docking_param_.final_hor_bias)
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
        if (std::fabs(relative_pos.z()) < docking_param_.final_ver_bias && horizontal_distance > 0.18f)
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
        // while (landing_target_pose_lock_.test_and_set(std::memory_order_acquire))
        //     ;
        if (is_landing_target_pose_updated_)
        {
            // is_replan_ = true; // TODO 是否需要立即重新规划，还是保持原来的重新规划频率？
            target_pos_.x() = landing_target_pose_.pose.position.x;
            target_pos_.y() = landing_target_pose_.pose.position.y;
            target_pos_.z() = landing_target_pose_.pose.position.z;
            target_vel_ = Eigen::Vector3d::Zero();
            //target_vel_.z() = -0.05;  //额外z轴速度
            target_q_.x() = landing_target_pose_.pose.orientation.x;
            target_q_.y() = landing_target_pose_.pose.orientation.y;
            target_q_.z() = landing_target_pose_.pose.orientation.z;
            target_q_.w() = landing_target_pose_.pose.orientation.w;


        }

        //如果最终降落阶段没识别到二维码，则抬高高度
        //如果没识别到不会进入这里，直接进入retry
        else
        {
            target_pos_.z() = uav_odom_pos_.z() + 0.25;
            ROS_INFO("LOSE TARGET--- DANGER");
        }

        ROS_INFO("fin:TARGET_POS:%.2f,%.2f,%.2f\n",target_pos_.x(),target_pos_.y(),target_pos_.z());
        if(PlanTrajectory());
        {
            is_replan_ = false;
            // 打印目标位姿
            // ROS_INFO("target x:%.2f,y:%.2f,z:%.2f", target_pos_.x(), target_pos_.y(), target_pos_.z());
        }
        //Pub_px4_cmd(target_pos_.x(),target_pos_.y(),target_pos_.z());

        break;
    }
    default:
        break;
    }
}

void OnboardUavFsm::RunDockingComplete()
{
    // 发送 Onboard::LAND_COMPLETE
    onboard_published_.flight_command = quadrotor_msgs::Onboard::DOCKING;
    onboard_published_.flight_status = quadrotor_msgs::Onboard::LAND_COMPLETE;
    PubOnboardMsg();

    // TODO 发送信号给traj_server，让其停止发布，停止发布心跳包
    if (uav_state_.armed == false)
    {
        perform_uav_disarm_ = false;
        ROS_INFO("\033[32mDOCKING_COMPLETE: Docking complete! UAV disarmed!\033[0m");
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
        if (LandingTargetPoseIsReceived(ros::Time::now()))
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
        //Pub_px4_cmd(target_pos_.x(),target_pos_.y(),target_pos_.z());
        if(PlanTrajectory());
        {
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
        if (LandingTargetPoseIsReceived(ros::Time::now()))
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

        //Pub_px4_cmd(target_pos_.x(),target_pos_.y(),target_pos_.z());
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

bool OnboardUavFsm::PlanTrajectory()
{
    if (!is_replan_ && !is_first_run_)
    {
        return true;
    }

    Eigen::MatrixXd start_state(3, 4);
    start_state.setZero();
    ros::Time replan_start_time = ros::Time::now(); // + ros::Duration(0.03)
    double replan_time = (replan_start_time - replan_start_time_).toSec();
    if (is_first_run_ || replan_time > poly_traj_.getTotalDuration())
    {
        start_state.col(0) = uav_odom_pos_;
        start_state.col(1) = uav_odom_vel_;
    }
    // else if (landing_state_ == LandingStates::FINAL_LANDING) // TODO 最终降落时，可以考虑用传感器数据
    // {
    //     start_state.col(0) = poly_traj_.getPos(replan_time);
    //     start_state.col(1) = poly_traj_.getVel(replan_time);
    //     start_state.col(2) = 1.2*poly_traj_.getAcc(replan_time);
    //     start_state.col(3) = poly_traj_.getJer(replan_time);
    // }
    else
    {
        start_state.col(0) = poly_traj_.getPos(replan_time);
        start_state.col(1) = poly_traj_.getVel(replan_time);
        start_state.col(2) = poly_traj_.getAcc(replan_time);
        start_state.col(3) = poly_traj_.getJer(replan_time);

        // // std::cout << "replan_pos " << (start_state.col(0) - uav_odom_pos_).norm() << std::endl;
        // // std::cout << "replan_vel " << start_state.col(1).transpose() << std::endl;
        // Eigen::Vector3d tail_vel0 = poly_traj_.getVel(poly_traj_.getTotalDuration());
        // // std::cout << "tail_vel " << tail_vel0.transpose() << std::endl;

        // // Eigen::Vector3d tail_pos = poly_traj_.getPos(replan_time);
        // // Eigen::Vector3d tail_vel = poly_traj_.getVel(replan_time);
        // Eigen::Vector3d tail_pos = uav_odom_pos_;
        // Eigen::Vector3d tail_vel;
        // tail_vel << uav_local_vel_.twist.linear.x, uav_local_vel_.twist.linear.y, uav_local_vel_.twist.linear.z;
        // vis_ptr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "cur_vel");              // 可视化末端速度
        // vis_ptr_->visualize_arrow(target_pos_, target_pos_ + 0.5 * tail_vel0, "tail_vel");      // 可视化末端速度
    }
    // is_first_run_ = false;
    // replan_start_time_ = replan_start_time;

    bool is_success = traj_opt_ptr_->generate_traj(start_state, target_pos_, target_vel_, target_q_, 10, poly_traj_);
    if (is_success)
    {
        is_first_run_ = false;
        replan_start_time_ = replan_start_time;
        //PubTrajectory函数将经过优化器优化后的轨迹poly_traj_发布
        PubTrajectory(replan_start_time_);

        //lc add
        traj_opt_ptr_->trans_bvp_traj(bvp_traj_);
        vis_ptr_->visualize_traj(bvp_traj_, "bvp_trajectory"); 

        vis_ptr_->visualize_traj(poly_traj_, "onboard_uav_trajectory");
    
        if (docking_state_ == DockingStates::LANDING)
        // if(landing_state_ == LandingStates::DESCEND_ABOVE_TARGET)
        {
            //std::cout << std::fixed << std::setprecision(2) << "[current] pos:" << uav_odom_pos_.transpose() << ";  ";
            //std::cout << std::fixed << std::setprecision(2) << "vel: " << uav_odom_vel_.transpose() << std::endl;

            //调试使用
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows;
            for (double dt = 0; dt < poly_traj_.getTotalDuration(); dt += 0.3)
            {
                Eigen::Vector3d pos = poly_traj_.getPos(dt);
                Eigen::Vector3d vel = poly_traj_.getVel(dt);
                // arrows.push_back(std::make_pair(pos, pos + 0.5 * vel));
                // 打印位置，保留两位小数
                //std::cout << std::fixed << std::setprecision(2) << "[target]  pos: " << pos.transpose() << ";  ";
                //std::cout << std::fixed << std::setprecision(2) << "vel: " << vel.transpose() << std::endl;
            }
            Eigen::Vector3d pos = poly_traj_.getPos(poly_traj_.getTotalDuration());
            Eigen::Vector3d vel = poly_traj_.getVel(poly_traj_.getTotalDuration());
            //std::cout << std::fixed << std::setprecision(2) << "[target]  pos: " << pos.transpose() << ";  ";
            //std::cout << std::fixed << std::setprecision(2) << "vel: " << vel.transpose() << std::endl;
            // vis_ptr_->visualize_arrows(arrows, "vel_arrows"); // 可视化速度
        }
    }
    return is_success;
}

// void OnboardUavFsm::PubTrajectory(const ros::Time& replan_start_time)
// {
//     quadrotor_msgs::PolyTraj traj_msg;
//     traj_msg.hover = false;
//     traj_msg.order = 5;
//     Eigen::VectorXd durs = poly_traj_.getDurations();
//     int piece_num = poly_traj_.getPieceNum();
//     traj_msg.duration.resize(piece_num);
//     traj_msg.coef_x.resize(6 * piece_num);
//     traj_msg.coef_y.resize(6 * piece_num);
//     traj_msg.coef_z.resize(6 * piece_num);
//     for (int i = 0; i < piece_num; ++i)
//     {
//         traj_msg.duration[i] = durs(i);
//         CoefficientMat cMat = poly_traj_[i].getCoeffMat();
//         int idx = 6 * i;
//         for (int j = 0; j < 6; ++j)
//         {
//             traj_msg.coef_x[idx + j] = cMat(0, j);
//             traj_msg.coef_y[idx + j] = cMat(1, j);
//             traj_msg.coef_z[idx + j] = cMat(2, j);
//         }
//     }
//     traj_msg.start_time = replan_start_time;
//     traj_msg.traj_id = traj_id_++;
//     traj_msg.yaw = 0.0; // TODO
//     trajectory_pub_.publish(traj_msg);
// }

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
        mavros_msgs::PositionTarget::IGNORE_YAW;
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        P_target.position.x = target_x;
        P_target.position.y = target_y;
        P_target.position.z = target_z;

        position_ctl_pub_.publish(P_target);
}

void OnboardUavFsm::Pub_Guidance_State()
{
    guidance_state_.header.stamp = ros::Time::now();
    guidance_state_.uav_id = "Sub-UAV";
    guidance_state_.Guidance_mode = rg_flag;
    switch (rg_flag)
    {
        case 1:
            guidance_state_.Guidance_mode_s = "GO_4points";
            break;
        case 2:
            guidance_state_.Guidance_mode_s = "GEO_EST";
            break;
        case 3:
            guidance_state_.Guidance_mode_s = "ITE_EST";
            break;
        default:
            break;
    }

    guidance_state_.geo_est_x = geo_est_c2d.x() + dock_r2m.x();
    guidance_state_.geo_est_y = geo_est_c2d.y() + dock_r2m.y();
    guidance_state_.geo_est_z = geo_est_c2d.z() + dock_r2m.z();

    guidance_state_.ite_est_x = ite_est_p.x();
    guidance_state_.ite_est_y = ite_est_p.y();
    guidance_state_.ite_est_z = ite_est_p.z();

    guidance_state_.ite_err_x = ite_c2d_q.x() - dock_r2m.x();
    guidance_state_.ite_err_y = ite_c2d_q.y() - dock_r2m.y();
    guidance_state_.ite_err_z = ite_c2d_q.z() - dock_r2m.z();

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