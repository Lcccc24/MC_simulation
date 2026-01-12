#include "vision_pose/landing_target_pose.hpp"

LandingTargetPose::LandingTargetPose(ros::NodeHandle &nh) : nh_(nh) //, tf_listener_(tf_buffer_)
{
    getParam();

    uav_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/Sub_UAV/mavros/imu/data_raw", 1, &LandingTargetPose::ImuCallback, this);
    uav_local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/Sub_UAV/mavros/local_position/pose", 1, &LandingTargetPose::LocalPosCallback, this);
    m_uav_local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/AVC/mavros/local_position/pose", 1, &LandingTargetPose::M_LocalPosCallback, this);
    tag_detection_sub_ = nh_.subscribe<apriltag_ros::AprilTagDetectionArray>(tag_param_.topic_name, 1, &LandingTargetPose::TagDetectionCallback, this);
    uwb_sub_ = nh_.subscribe<std_msgs::Float64>("/fake_uwb_distance", 10, &LandingTargetPose::UwbDistanceCallback, this);
    landing_target_pose_raw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_pose_raw", 1);
    landing_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_pose/ESKF", 1);
    landing_relative_odom_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_relative_odom", 1);
    rg_est_mother_sub_ = nh_.subscribe<quadrotor_msgs::GuidanceState>("/remote_ctrl/state", 1, &LandingTargetPose::RgEstMotherCallback, this);

    eskf_timer_ = nh_.createTimer(ros::Duration(1.0 / eskf_param_.eskf_hz), &LandingTargetPose::EskfTimerCallback, this);

    Init();
}

LandingTargetPose::~LandingTargetPose()
{
}

void LandingTargetPose::getParam()
{
    nh_.param("eskf/eskf_hz", eskf_param_.eskf_hz, 200);
    nh_.param("eskf/gyro_var", eskf_param_.gyro_var, 0.01);
    nh_.param("eskf/acce_var", eskf_param_.acce_var, 0.01);
    nh_.param("eskf/bias_gyro_var", eskf_param_.bias_gyro_var, 0.0001);
    nh_.param("eskf/bias_acce_var", eskf_param_.bias_acce_var, 0.0001);
    nh_.param("eskf/bias_gyro_x", eskf_param_.gyro_bias(0), 0.0);
    nh_.param("eskf/bias_gyro_y", eskf_param_.gyro_bias(1), 0.0);
    nh_.param("eskf/bias_gyro_z", eskf_param_.gyro_bias(2), 0.0);
    nh_.param("eskf/bias_acce_x", eskf_param_.acce_bias(0), 0.0);
    nh_.param("eskf/bias_acce_y", eskf_param_.acce_bias(1), 0.0);
    nh_.param("eskf/bias_acce_z", eskf_param_.acce_bias(2), 0.0);
    nh_.param("eskf/update_bias_gyro", eskf_param_.update_bias_gyro, true);
    nh_.param("eskf/update_bias_acce", eskf_param_.update_bias_acce, true);
    nh_.param("eskf/vision_xy_noise", eskf_param_.vision_xy_noise, 0.01);
    nh_.param("eskf/vision_z_noise", eskf_param_.vision_z_noise, 0.01);
    nh_.param("eskf/vision_roll_pitch_noise", eskf_param_.vision_roll_pitch_noise, 0.01);
    nh_.param("eskf/vision_yaw_noise", eskf_param_.vision_yaw_noise, 0.01);
    nh_.param("eskf/uwb_noise", eskf_param_.uwb_noise, 0.01);
    nh_.param("eskf/relative_position_xy_noise", eskf_param_.relative_position_xy_noise, 0.01);
    nh_.param("eskf/relative_position_z_noise", eskf_param_.relative_position_z_noise, 0.01);
    nh_.param("eskf/gravity", eskf_param_.gravity, 9.8);

    nh_.param("tag/topic_name", tag_param_.topic_name, std::string("/tag_detections"));
    nh_.param("tag/tag_frame_id", tag_param_.tag_frame_id, std::string("tag36h11"));
    nh_.param("tag/tag_valid_distance", tag_param_.tag_valid_distance, 0.0);
    nh_.param("frame/camera_frame_id", frame_param_.camera_frame_id, std::string("camera_link"));

    frame_param_.body_to_camera_p = Eigen::Vector3d(-0.08, 0.0, 0.0);
    //绕x旋转180，绕z旋转90
    frame_param_.body_to_camera_q = Eigen::Quaterniond(0.0, 0.707, -0.707, 0.0);
    frame_param_.body_to_camera_q.normalize();
    frame_param_.uav0_to_tag_p = Eigen::Vector3d(0.0, 0.0, 0.1);
    //绕z轴旋转-90度
    frame_param_.uav0_to_tag_q = Eigen::Quaterniond(0.707, 0.0, 0.0, -0.707);
    frame_param_.uav0_to_tag_q.normalize();
    frame_param_.uav0_to_landing_p = Eigen::Vector3d(0.0, 0.0, 0.15);
    frame_param_.uav0_to_landing_q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    frame_param_.uav0_to_landing_q.normalize();
}

void LandingTargetPose::UpdateParam()
{
    // 读取参数，当参数发生变化时更新eskf参数
    static eskf_param eskf_param_temp = eskf_param_;

    nh_.getParam("eskf/gyro_var", eskf_param_temp.gyro_var);
    nh_.getParam("eskf/acce_var", eskf_param_temp.acce_var);
    nh_.getParam("eskf/bias_gyro_var", eskf_param_temp.bias_gyro_var);
    nh_.getParam("eskf/bias_acce_var", eskf_param_temp.bias_acce_var);
    nh_.getParam("eskf/bias_gyro_x", eskf_param_temp.gyro_bias(0));
    nh_.getParam("eskf/bias_gyro_y", eskf_param_temp.gyro_bias(1));
    nh_.getParam("eskf/bias_gyro_z", eskf_param_temp.gyro_bias(2));
    nh_.getParam("eskf/bias_acce_x", eskf_param_temp.acce_bias(0));
    nh_.getParam("eskf/bias_acce_y", eskf_param_temp.acce_bias(1));
    nh_.getParam("eskf/bias_acce_z", eskf_param_temp.acce_bias(2));
    nh_.getParam("eskf/vision_xy_noise", eskf_param_temp.vision_xy_noise);
    nh_.getParam("eskf/vision_z_noise", eskf_param_temp.vision_z_noise);
    nh_.getParam("eskf/vision_roll_pitch_noise", eskf_param_temp.vision_roll_pitch_noise);
    nh_.getParam("eskf/vision_yaw_noise", eskf_param_temp.vision_yaw_noise);
    nh_.getParam("eskf/uwb_noise", eskf_param_temp.uwb_noise);
    nh_.getParam("eskf/relative_position_xy_noise", eskf_param_temp.relative_position_xy_noise);
    nh_.getParam("eskf/relative_position_z_noise", eskf_param_temp.relative_position_z_noise);

    if (eskf_param_temp.gyro_var != eskf_param_.gyro_var ||
        eskf_param_temp.acce_var != eskf_param_.acce_var ||
        eskf_param_temp.bias_gyro_var != eskf_param_.bias_gyro_var ||
        eskf_param_temp.bias_acce_var != eskf_param_.bias_acce_var ||
        eskf_param_temp.gyro_bias(0) != eskf_param_.gyro_bias(0) ||
        eskf_param_temp.gyro_bias(1) != eskf_param_.gyro_bias(1) ||
        eskf_param_temp.gyro_bias(2) != eskf_param_.gyro_bias(2) ||
        eskf_param_temp.acce_bias(0) != eskf_param_.acce_bias(0) ||
        eskf_param_temp.acce_bias(1) != eskf_param_.acce_bias(1) ||
        eskf_param_temp.acce_bias(2) != eskf_param_.acce_bias(2) ||
        eskf_param_temp.vision_xy_noise != eskf_param_.vision_xy_noise ||
        eskf_param_temp.vision_z_noise != eskf_param_.vision_z_noise ||
        eskf_param_temp.vision_roll_pitch_noise != eskf_param_.vision_roll_pitch_noise ||
        eskf_param_temp.vision_yaw_noise != eskf_param_.vision_yaw_noise ||
        eskf_param_temp.uwb_noise != eskf_param_.uwb_noise ||
        eskf_param_temp.relative_position_xy_noise != eskf_param_.relative_position_xy_noise ||
        eskf_param_temp.relative_position_z_noise != eskf_param_.relative_position_z_noise)
    {
        eskf_param_ = eskf_param_temp;
        eskf_init_flag_ = false;
    }
}

void LandingTargetPose::SetEskfParam()
{
    static ESKFD::Options options;
    options.imu_dt_ = 1.0 / eskf_param_.eskf_hz;
    options.gyro_var_ = eskf_param_.gyro_var;
    options.acce_var_ = eskf_param_.acce_var;
    options.bias_gyro_var_ = eskf_param_.bias_gyro_var;
    options.bias_acce_var_ = eskf_param_.bias_acce_var;
    options.vision_xy_noise_ = eskf_param_.vision_xy_noise;
    options.vision_z_noise_ = eskf_param_.vision_z_noise;
    options.vision_roll_pitch_noise_ = eskf_param_.vision_roll_pitch_noise;
    options.vision_yaw_noise_ = eskf_param_.vision_yaw_noise;
    options.relative_position_xy_noise_ = eskf_param_.relative_position_xy_noise;
    options.relative_position_z_noise_ = eskf_param_.relative_position_z_noise;
    options.uwb_noise_ = eskf_param_.uwb_noise;
    options.update_bias_gyro_ = eskf_param_.update_bias_gyro;
    options.update_bias_acce_ = eskf_param_.update_bias_acce;

    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -eskf_param_.gravity);

    eskf_.SetInitialConditions(options, eskf_param_.gyro_bias, eskf_param_.acce_bias, gravity);
}

void LandingTargetPose::Init()
{
    imu_init_flag_ = false;

    // 视觉监测状态
    get_new_landing_target_ = false;
    last_vision_time_ = 0.0;
    vision_timeout_ = 2.0;  // 1秒超时
    vision_valid_ = false;

    // uwb监测状态
    uwb_distance_ = 0.0;
    last_uwb_time_ = 0.0;
    uwb_timeout_ = 1.0;
    uwb_valid_ = false;
    get_new_uwb_ = false;

    // 相对位置监测状态
    get_new_relative_position_ = false;
    last_relative_position_time_ = 0.0;
    relative_position_timeout_ = 1.0;
    relative_position_valid_ = false;
    
    eskf_init_flag_ = false;
    eskf_enabled_ = false;

    SetEskfParam();

    eskf_.SetConstant(frame_param_.body_to_camera_p, frame_param_.body_to_camera_q, frame_param_.uav0_to_tag_p, frame_param_.uav0_to_tag_q);
}

void LandingTargetPose::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ = *msg;
    imu_init_flag_ = true;
}

void LandingTargetPose::LocalPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_local_pos_ = *msg;
}

void LandingTargetPose::M_LocalPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_uav_local_pos_ = *msg;
}

void LandingTargetPose::RgEstMotherCallback(const quadrotor_msgs::GuidanceState::ConstPtr &msg)
{
    rg_msg_ = *msg;
    if (msg->have_solution == true) {
        rg_est_mother_pos_ = Eigen::Vector3d(msg->geo_est_x, msg->geo_est_y, msg->geo_est_z);
        mother_pos_offset_.x() = m_uav_local_pos_.pose.position.x - rg_est_mother_pos_.x();
        mother_pos_offset_.y() = m_uav_local_pos_.pose.position.y - rg_est_mother_pos_.y();
        mother_pos_offset_.z() = m_uav_local_pos_.pose.position.z - rg_est_mother_pos_.z();
        have_mother_pos_offset_ = true;
    }
}

void LandingTargetPose::TagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    const double now = ros::Time::now().toSec();

    // ---- 随机“偶发性丢失”模型（泊松到达 + 均匀时长）----
    // 目标：10 秒内平均 2~3 次 → 取平均到达率 λ ≈ 0.25 / s（10s 里期望 2.5 次）
    // 持续时长 ~ U[0.1, 0.3] s
    static std::mt19937 rng(std::random_device{}());

    // 状态：当前是否在丢失窗口、下次丢失的开始时刻、当前丢失结束时刻
    static bool loss_active = false;
    static double loss_end_time = 0.0;
    static double next_loss_start_time = 0.0;

    // 分布：指数分布用于下次到达间隔；均匀分布用于丢失持续时间
    static std::exponential_distribution<double> inter_arrival(/*lambda*/ 0.4); // 平均 4s 到达一次
    static std::uniform_real_distribution<double> loss_duration(0.1, 0.6);      // 丢失 0.1~0.3s

    // 首次初始化 next_loss_start_time
    if (next_loss_start_time == 0.0) {
        next_loss_start_time = now + inter_arrival(rng);
    }

    // 若到了下次丢失开始点且当前不在丢失中 → 触发一次丢失
    if (!loss_active && now >= next_loss_start_time) {
        loss_active = true;
        const double dur = loss_duration(rng);
        loss_end_time = now + dur;
        // 下一次丢失的“开始时刻”也随机排程（指数分布的到达间隔，独立于本次持续时间）
        next_loss_start_time = now + inter_arrival(rng);
        ROS_WARN("[Simulation] Vision LOSS start, duration=%.3fs", dur);
    }

    // 若在丢失窗口内
    if (loss_active) {
        if (now < loss_end_time) {
            ROS_WARN_THROTTLE(0.5, "[Simulation] Vision data suppressed (random loss...)");
            return;  // 丢失期间直接退出
        } else {
            // 丢失结束
            loss_active = false;
            ROS_INFO("[Simulation] Vision recovered");
        }
    }

    if (msg->detections.size() == 0)
    {
        return;
    }

    std::set<int> bundle_ids = {0, 1};

    for (const auto &detection : msg->detections)
    {
        bool is_correct_bundle = true;
        for (const auto &id_value : detection.id) {
            if (bundle_ids.find(id_value) == bundle_ids.end()) {
                is_correct_bundle = false;
                break;
            }
        }

        if (!is_correct_bundle) {
            ROS_INFO("incorrect bundle id");
            continue;
        }

        if (std::fabs(detection.pose.pose.pose.position.z) > tag_param_.tag_valid_distance){
            ROS_INFO("tag pose z is too far");
            continue;
        }

        // if (!IsTagPoseValid()) {
        //     ROS_INFO("tag pose is not valid");
        //     continue;
        // }

        tag_pose_.header = detection.pose.header;
        tag_pose_.pose = detection.pose.pose.pose;

        // 标记视觉数据有效
        last_vision_time_ = ros::Time::now().toSec();
        get_new_landing_target_ = true;
        vision_valid_ = true;
    
        // 视觉恢复时重新激活ESKF
        if (!eskf_enabled_) {
            ROS_INFO("Vision recovered, activating ESKF");
            eskf_enabled_ = true;
        }

        break;
    }

}

void LandingTargetPose::UwbDistanceCallback(const std_msgs::Float64::ConstPtr &msg) {
    uwb_distance_ = msg->data;
    last_uwb_time_ = ros::Time::now().toSec();
    uwb_valid_ = true;
    get_new_uwb_ = true;
}

bool LandingTargetPose::IsTagPoseValid()
{
    static int eskf_outlier_reject_count = 0;
    static bool is_position_valid = true;
    static bool is_orientation_valid = true;

    double x_d = std::fabs(last_tag_pose_.pose.position.x - tag_pose_.pose.position.x);
    double y_d = std::fabs(last_tag_pose_.pose.position.y - tag_pose_.pose.position.y);
    double z_d = std::fabs(last_tag_pose_.pose.position.z - tag_pose_.pose.position.z);
    double q_x_d = std::fabs(last_tag_pose_.pose.orientation.x - tag_pose_.pose.orientation.x);
    double q_y_d = std::fabs(last_tag_pose_.pose.orientation.y - tag_pose_.pose.orientation.y);
    double q_z_d = std::fabs(last_tag_pose_.pose.orientation.z - tag_pose_.pose.orientation.z);
    double q_w_d = std::fabs(last_tag_pose_.pose.orientation.w - tag_pose_.pose.orientation.w);

    if ((x_d > 0.5 || y_d > 0.5 || z_d > 0.5 || q_x_d > 0.5 || q_y_d > 0.1 || q_z_d > 0.1 || q_w_d > 0.1) && eskf_outlier_reject_count < 10)
    {
        eskf_outlier_reject_count++;
        return false;
    }

    last_tag_pose_ = tag_pose_;

    eskf_outlier_reject_count = 0;

    return true;
}

void LandingTargetPose::UpdateRelativePosition() {
    if (uav_local_pos_.header.stamp.toSec() > 0 && m_uav_local_pos_.header.stamp.toSec() > 0) {
        double time_diff = std::abs(uav_local_pos_.header.stamp.toSec() - m_uav_local_pos_.header.stamp.toSec());

        if (time_diff < 0.05 && have_mother_pos_offset_) {  // 50ms以内的同步误差可以接受
            // 计算两架飞机在全局坐标系下的位置差
            Eigen::Vector3d uav_pos(
                uav_local_pos_.pose.position.x,
                uav_local_pos_.pose.position.y,
                uav_local_pos_.pose.position.z
            );
            
            Eigen::Vector3d m_uav_pos(
                m_uav_local_pos_.pose.position.x - mother_pos_offset_.x(),
                m_uav_local_pos_.pose.position.y - mother_pos_offset_.y(),
                m_uav_local_pos_.pose.position.z - mother_pos_offset_.z()
            );

            Eigen::Vector3d rel_pos_global = uav_pos - m_uav_pos;

            // 获取大飞机的姿态（从全局坐标系到大飞机机体坐标系的旋转）
            Eigen::Quaterniond m_uav_q(
                m_uav_local_pos_.pose.orientation.w,
                m_uav_local_pos_.pose.orientation.x,
                m_uav_local_pos_.pose.orientation.y,
                m_uav_local_pos_.pose.orientation.z
            );

            m_uav_q.normalize();
            // 将相对位置从全局坐标系转换到大飞机机体坐标系
            // 注意：状态量p_ms是小飞机在大飞机机体坐标系下的位置
            Eigen::Vector3d rel_pos_body = m_uav_q.inverse() * rel_pos_global;
            ROS_INFO("Relative position: %f, %f, %f", rel_pos_body.x(), rel_pos_body.y(), rel_pos_body.z());

            double distance = rel_pos_body.norm();
            relative_pos_ = rel_pos_body;
            last_relative_position_time_ = ros::Time::now().toSec();
            relative_position_valid_ = true;
            get_new_relative_position_ = true;
            // landing_target_relative_odom_.pose.position.x = rel_pos_body.x();
            // landing_target_relative_odom_.pose.position.y = rel_pos_body.y();
            // landing_target_relative_odom_.pose.position.z = rel_pos_body.z();
            // landing_target_relative_odom_.header.stamp = ros::Time::now();
            // landing_relative_odom_pub_.publish(landing_target_relative_odom_);

            static Eigen::Vector3d p_ml = frame_param_.uav0_to_landing_p;
            static Sophus::SO3d R_ml = Sophus::SO3d(frame_param_.uav0_to_landing_q);

            Eigen::Vector3d p_ms = rel_pos_body;
            Sophus::SO3d R_ms = Sophus::SO3d(m_uav_q);

            Eigen::Vector3d p_es = uav_pos;
            Eigen::Quaterniond q_es = Eigen::Quaterniond(uav_local_pos_.pose.orientation.w, uav_local_pos_.pose.orientation.x, uav_local_pos_.pose.orientation.y, uav_local_pos_.pose.orientation.z);
            q_es.normalize();
            Sophus::SO3d R_es = Sophus::SO3d(q_es);

            Eigen::Vector3d p_el = p_es + (R_es * R_ms.inverse()).matrix() * (p_ml - p_ms);
            Sophus::SO3d R_el = R_es * R_ms.inverse() * R_ml;

            landing_target_relative_odom_.header.stamp = ros::Time::now();
            landing_target_relative_odom_.header.frame_id = "map";
            landing_target_relative_odom_.pose.position.x = p_el.x();
            landing_target_relative_odom_.pose.position.y = p_el.y();
            landing_target_relative_odom_.pose.position.z = p_el.z();
            landing_relative_odom_pub_.publish(landing_target_relative_odom_);
        }
    }
}

void LandingTargetPose::EskfTimerCallback(const ros::TimerEvent &event)
{
    const double current_time = ros::Time::now().toSec();

    // 检查IMU初始化状态
    if (!imu_init_flag_) {
        return;
    }

    // 由于相对位置无回调 先在定时器循环中更新相对位置信息
    if (eskf_init_flag_) {
        UpdateRelativePosition();
    }

    // 视觉超时检测
    if (eskf_init_flag_ && vision_valid_ && (current_time - last_vision_time_ > vision_timeout_)) 
    {
        ROS_ERROR_STREAM("[ESKF] Vision timeout! Last update: " 
                        << current_time - last_vision_time_ << "s ago");
        vision_valid_ = false;
    }

    // UWB超时检测
    if (eskf_init_flag_ && uwb_valid_ && (current_time - last_uwb_time_ > uwb_timeout_)) 
    {
        ROS_ERROR_STREAM("[ESKF] UWB timeout! Last update: " 
                        << current_time - last_uwb_time_ << "s ago");
        uwb_valid_ = false;
    }

    // 相对位置超时检测
    if (eskf_init_flag_ && relative_position_valid_ && (current_time - last_relative_position_time_ > relative_position_timeout_)) 
    {
        ROS_ERROR_STREAM("[ESKF] Relative position timeout! Last update: " 
                        << current_time - last_relative_position_time_ << "s ago");
        relative_position_valid_ = false;
    }

    // 根据观测传感器状态决定是否激活ESKF
    if (eskf_init_flag_) {
        if (vision_valid_ || relative_position_valid_) {
            eskf_enabled_ = true;
        } else {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[ESKF] All sensors lost! Resetting ESKF to wait for recovery.");
            eskf_enabled_ = false;
            eskf_init_flag_ = false;
        }
    }

    //以一定频率更新参数
    static int update_param_count = 0;
    if (update_param_count++ > 1 * eskf_param_.eskf_hz) {
        UpdateParam();
        update_param_count = 0;
    }

    //Apritag原始数据发布 速率取决于视觉数据更新频率 不受定时器频率影响
    if (get_new_landing_target_) {
        CalculateLandingTargetPoseRaw();
        landing_target_pose_raw_pub_.publish(landing_target_pose_raw_);
    }

    // ESKF初始化 用第一帧视觉数据
    if (!eskf_init_flag_ && get_new_landing_target_) {
        eskf_.Reset();
        eskf_enabled_ = true;
        get_new_landing_target_ = false;
        last_tag_pose_ = tag_pose_;
        eskf_.ObserveVision(tag_pose_);
        eskf_init_time_ = current_time;
        eskf_init_flag_ = true;
        
        ROS_INFO("[ESKF] Initialized with first vision data");
    }

    // imu预测
    if (eskf_init_flag_) { 
        try {
            eskf_.Predict(imu_);
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("ESKF prediction failed: " << e.what());
        }
    }

    // Apritag视觉观测
    if (get_new_landing_target_) {
        get_new_landing_target_ = false;
        
        if (eskf_init_flag_ && eskf_enabled_ && vision_valid_) {
            try {
                eskf_.ObserveVision(tag_pose_);
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("ESKF observation failed: " << e.what());
            }
        }
    }

    // 相对位置观测
    if (get_new_relative_position_) {
        get_new_relative_position_ = false;
        
        if (eskf_init_flag_ && eskf_enabled_ && relative_position_valid_) {
            try {
                eskf_.ObserveRelativePosition(relative_pos_);
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("ESKF relative position observation failed: " << e.what());
            }
        }
    }

    // Uwb观测
    if (get_new_uwb_) {
        get_new_uwb_ = false;
    
        if (eskf_init_flag_ && eskf_enabled_ && uwb_valid_) {
            try {
                eskf_.ObserveUWB(uwb_distance_);
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("ESKF UWB observation failed: " << e.what());
            }
        }
    }

    // 发布输出
    if (eskf_enabled_ && eskf_init_flag_) {
        // 正常发布数据
        nominal_state_ = eskf_.GetNominalSE3();
        CalculateLandingTargetPose();
        landing_target_pose_pub_.publish(landing_target_pose_);
    } else if (eskf_init_flag_) {  // 已初始化但被禁用
        static ros::Time last_warn_time = ros::Time(0);
        if ((ros::Time::now() - last_warn_time).toSec() > 1.0) {
            ROS_WARN_THROTTLE(1.0, "[ESKF] Output disabled (no valid sensors)");
            last_warn_time = ros::Time::now();
        }
    }
}


void LandingTargetPose::CalculateLandingTargetPose()
{
    static Eigen::Vector3d p_ms = Eigen::Vector3d::Zero();
    static Eigen::Vector3d p_es = Eigen::Vector3d::Zero();
    static Eigen::Vector3d p_el = Eigen::Vector3d::Zero();
    static Eigen::Vector3d p_ml = frame_param_.uav0_to_landing_p;
    static Sophus::SO3d R_el, R_es, R_ms;
    // static Eigen::Matrix3d R_es = Eigen::Matrix3d::Identity();
    // static Eigen::Matrix3d R_el = Eigen::Matrix3d::Identity();
    // static Eigen::Matrix3d R_ct = Eigen::Matrix3d::Identity();
    static Eigen::Quaterniond q_el = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    static Eigen::Quaterniond q_ct = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    static Eigen::Quaterniond q_es = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    static Sophus::SO3d R_sc = Sophus::SO3d(frame_param_.body_to_camera_q);
    static Sophus::SO3d R_mt = Sophus::SO3d(frame_param_.uav0_to_tag_q);
    static Sophus::SO3d R_ml = Sophus::SO3d(frame_param_.uav0_to_landing_q);

    p_ms = nominal_state_.translation();
    R_ms = nominal_state_.so3();

    p_es = Eigen::Vector3d(uav_local_pos_.pose.position.x, uav_local_pos_.pose.position.y, uav_local_pos_.pose.position.z);
    q_es = Eigen::Quaterniond(uav_local_pos_.pose.orientation.w, uav_local_pos_.pose.orientation.x, uav_local_pos_.pose.orientation.y, uav_local_pos_.pose.orientation.z);
    q_es.normalize();
    R_es = Sophus::SO3d(q_es);

    p_el = p_es + (R_es * R_ms.inverse()).matrix() * (p_ml - p_ms);
    R_el = R_es * R_ms.inverse() * R_ml;

    landing_target_pose_.header.stamp = ros::Time::now();
    landing_target_pose_.header.frame_id = "map";
    landing_target_pose_.pose.position.x = p_el.x();
    landing_target_pose_.pose.position.y = p_el.y();
    landing_target_pose_.pose.position.z = p_el.z();
    q_el = Eigen::Quaterniond(R_el.unit_quaternion());
    q_el.normalize();
    landing_target_pose_.pose.orientation.w = q_el.w();
    landing_target_pose_.pose.orientation.x = q_el.x();
    landing_target_pose_.pose.orientation.y = q_el.y();
    landing_target_pose_.pose.orientation.z = q_el.z();
    ROS_INFO("p_ms:XYZ:%f, %f, %f", p_ms.x(), p_ms.y(), p_ms.z());
}

void LandingTargetPose::CalculateLandingTargetPoseRaw()
{
    static Eigen::Vector3d p_es = Eigen::Vector3d::Zero();
    static Eigen::Vector3d p_sc = frame_param_.body_to_camera_p;
    static Eigen::Vector3d p_ml = frame_param_.uav0_to_landing_p;
    static Eigen::Vector3d p_mt = frame_param_.uav0_to_tag_p;
    static Eigen::Vector3d p_ct = Eigen::Vector3d::Zero();
    static Eigen::Vector3d p_el = Eigen::Vector3d::Zero();
    static Sophus::SO3d R_el, R_es, R_ct;
    // static Eigen::Matrix3d R_el = Eigen::Matrix3d::Identity();
    // static Eigen::Matrix3d R_es = Eigen::Matrix3d::Identity();
    static Eigen::Quaterniond q_el = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    static Eigen::Quaterniond q_es = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    // static Eigen::Matrix3d R_ct = Eigen::Matrix3d::Identity();
    static Eigen::Quaterniond q_ct = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    static Sophus::SO3d R_sc = Sophus::SO3d(frame_param_.body_to_camera_q);
    static Sophus::SO3d R_mt = Sophus::SO3d(frame_param_.uav0_to_tag_q);
    static Sophus::SO3d R_ml = Sophus::SO3d(frame_param_.uav0_to_landing_q);

    //当前位姿
    p_es = Eigen::Vector3d(uav_local_pos_.pose.position.x, uav_local_pos_.pose.position.y, uav_local_pos_.pose.position.z);
    q_es = Eigen::Quaterniond(uav_local_pos_.pose.orientation.w, uav_local_pos_.pose.orientation.x, uav_local_pos_.pose.orientation.y, uav_local_pos_.pose.orientation.z);
    q_es.normalize();
    R_es = Sophus::SO3d(q_es);

    //Apritag标签位姿
    p_ct = Eigen::Vector3d(tag_pose_.pose.position.x, tag_pose_.pose.position.y, tag_pose_.pose.position.z);

    // p_ct.x() += 0.03 * rand() / double(RAND_MAX);
    // p_ct.y() += 0.03 * rand() / double(RAND_MAX);
    // p_ct.z() += 0.03 * rand() / double(RAND_MAX);

    q_ct = Eigen::Quaterniond(tag_pose_.pose.orientation.w, tag_pose_.pose.orientation.x, tag_pose_.pose.orientation.y, tag_pose_.pose.orientation.z);
    q_ct.normalize();
    R_ct = Sophus::SO3d(q_ct);

    /*
    世界坐标系 (Earth Frame)
        ↑ R_es, p_es
    小飞机机体坐标系 (Sub-UAV Frame)
        ↑ R_sc, p_sc
    相机坐标系 (Camera Frame)
        ↑ R_ct, p_ct
    Tag坐标系 (Tag Frame)
        ↑ R_mt⁻¹, (p_ml - p_mt)
    大飞机机体坐标系 (Main UAV Frame)
        ↑ p_ml
    降落点 (Landing Point)
    */

    p_el = p_es + R_es.matrix() * (p_sc + R_sc.matrix() * (p_ct + (R_ct * R_mt.inverse()).matrix() * (p_ml - p_mt)));
    R_el = R_es * R_sc * R_ct * R_mt.inverse() * R_ml;

    landing_target_pose_raw_.header.stamp = tag_pose_.header.stamp;
    landing_target_pose_raw_.header.frame_id = "map";
    landing_target_pose_raw_.pose.position.x = p_el.x();
    landing_target_pose_raw_.pose.position.y = p_el.y();
    landing_target_pose_raw_.pose.position.z = p_el.z();
    q_el = Eigen::Quaterniond(R_el.unit_quaternion());
    q_el.normalize();
    landing_target_pose_raw_.pose.orientation.w = q_el.w();
    landing_target_pose_raw_.pose.orientation.x = q_el.x();
    landing_target_pose_raw_.pose.orientation.y = q_el.y();
    landing_target_pose_raw_.pose.orientation.z = q_el.z();
}
