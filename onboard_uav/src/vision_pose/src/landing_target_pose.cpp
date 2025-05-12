#include "vision_pose/landing_target_pose.hpp"

LandingTargetPose::LandingTargetPose(ros::NodeHandle &nh) : nh_(nh) //, tf_listener_(tf_buffer_)
{
    getParam();

    uav_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/Sub_UAV/mavros/imu/data_raw", 1, &LandingTargetPose::ImuCallback, this);
    uav0_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/AVC/mavros/imu/data_raw", 1, &LandingTargetPose::Imu0Callback, this);
    uav_local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/Sub_UAV/mavros/local_position/pose", 1, &LandingTargetPose::LocalPosCallback, this);
    tag_detection_sub_ = nh_.subscribe<apriltag_ros::AprilTagDetectionArray>(tag_param_.topic_name, 1, &LandingTargetPose::TagDetectionCallback, this);
    // gazebo_true_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &LandingTargetPose::GazeboTrueCallback, this);
    landing_target_pose_raw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_pose_raw", 1);
    landing_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_pose/ESKF", 1);

    eskf_timer_ = nh_.createTimer(ros::Duration(1.0 / eskf_param_.eskf_hz), &LandingTargetPose::EskfTimerCallback, this);

    Init();
}

LandingTargetPose::~LandingTargetPose()
{
}

void LandingTargetPose::getParam()
{
    nh_.param("eskf/eskf_hz", eskf_param_.eskf_hz, 200);
    nh_.param("eskf/use_imu0", eskf_param_.use_imu0, true);
    nh_.param("eskf/gyro_var", eskf_param_.gyro_var, 0.01);
    nh_.param("eskf/acce_var", eskf_param_.acce_var, 0.01);
    nh_.param("eskf/bias_gyro_var", eskf_param_.bias_gyro_var, 0.0001);
    nh_.param("eskf/bias_acce_var", eskf_param_.bias_acce_var, 0.0001);
    nh_.param("eskf/bias_acce_m_var", eskf_param_.bias_acce_m_var, 0.0001);
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
    nh_.param("eskf/gravity", eskf_param_.gravity, 9.8);

    nh_.param("tag/topic_name", tag_param_.topic_name, std::string("/tag_detections"));
    nh_.param("tag/tag_frame_id", tag_param_.tag_frame_id, std::string("tag36h11"));
    // nh_.param("tag/tag_to_landing_x", tag_param_.tag_to_landing_x, 0.0);
    // nh_.param("tag/tag_to_landing_y", tag_param_.tag_to_landing_y, 0.0);
    // nh_.param("tag/tag_to_landing_z", tag_param_.tag_to_landing_z, 0.0);
    // nh_.param("tag/tag_to_landing_yaw", tag_param_.tag_to_landing_yaw, 0.0);
    nh_.param("tag/tag_valid_distance", tag_param_.tag_valid_distance, 0.0);

    nh_.param("frame/camera_frame_id", frame_param_.camera_frame_id, std::string("camera_link"));
    // nh_.param("frame/body_to_camera_p_x", frame_param_.body_to_camera_p.x(), 0.0);
    // nh_.param("frame/body_to_camera_p_y", frame_param_.body_to_camera_p.y(), 0.0);
    // nh_.param("frame/body_to_camera_p_z", frame_param_.body_to_camera_p.z(), 0.0);
    // nh_.param("frame/body_to_camera_q_x", frame_param_.body_to_camera_q.x(), 0.0);
    // nh_.param("frame/body_to_camera_q_y", frame_param_.body_to_camera_q.y(), 0.0);
    // nh_.param("frame/body_to_camera_q_z", frame_param_.body_to_camera_q.z(), 0.0);
    // nh_.param("frame/body_to_camera_q_w", frame_param_.body_to_camera_q.w(), 1.0);
    // nh_.param("frame/uav0_to_tag_p_x", frame_param_.uav0_to_tag_p.x(), 0.0);
    // nh_.param("frame/uav0_to_tag_p_y", frame_param_.uav0_to_tag_p.y(), 0.0);
    // nh_.param("frame/uav0_to_tag_p_z", frame_param_.uav0_to_tag_p.z(), 0.0);
    // nh_.param("frame/uav0_to_tag_q_x", frame_param_.uav0_to_tag_q.x(), 0.0);
    // nh_.param("frame/uav0_to_tag_q_y", frame_param_.uav0_to_tag_q.y(), 0.0);
    // nh_.param("frame/uav0_to_tag_q_z", frame_param_.uav0_to_tag_q.z(), 0.0);
    // nh_.param("frame/uav0_to_tag_q_w", frame_param_.uav0_to_tag_q.w(), 1.0);
    frame_param_.body_to_camera_p = Eigen::Vector3d(-0.08, 0.0, 0.0);
    //绕z旋转90，绕x旋转180
    frame_param_.body_to_camera_q = Eigen::Quaterniond(0.0, 0.707, -0.707, 0.0);
    frame_param_.body_to_camera_q.normalize();
    frame_param_.uav0_to_tag_p = Eigen::Vector3d(0.0, 0.0, 0.1);
    //绕z轴旋转-90度
    frame_param_.uav0_to_tag_q = Eigen::Quaterniond(0.707, 0.0, 0.0, -0.707);
    frame_param_.uav0_to_tag_q.normalize();
    frame_param_.uav0_to_landing_p = Eigen::Vector3d(0.0, 0.0, 0.1);
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
    nh_.getParam("eskf/bias_acce_m_var", eskf_param_temp.bias_acce_m_var);
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

    if (eskf_param_temp.gyro_var != eskf_param_.gyro_var ||
        eskf_param_temp.acce_var != eskf_param_.acce_var ||
        eskf_param_temp.bias_gyro_var != eskf_param_.bias_gyro_var ||
        eskf_param_temp.bias_acce_var != eskf_param_.bias_acce_var ||
        eskf_param_temp.bias_acce_m_var != eskf_param_.bias_acce_m_var ||
        eskf_param_temp.gyro_bias(0) != eskf_param_.gyro_bias(0) ||
        eskf_param_temp.gyro_bias(1) != eskf_param_.gyro_bias(1) ||
        eskf_param_temp.gyro_bias(2) != eskf_param_.gyro_bias(2) ||
        eskf_param_temp.acce_bias(0) != eskf_param_.acce_bias(0) ||
        eskf_param_temp.acce_bias(1) != eskf_param_.acce_bias(1) ||
        eskf_param_temp.acce_bias(2) != eskf_param_.acce_bias(2) ||
        eskf_param_temp.vision_xy_noise != eskf_param_.vision_xy_noise ||
        eskf_param_temp.vision_z_noise != eskf_param_.vision_z_noise ||
        eskf_param_temp.vision_roll_pitch_noise != eskf_param_.vision_roll_pitch_noise ||
        eskf_param_temp.vision_yaw_noise != eskf_param_.vision_yaw_noise)
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
    options.bias_acce_m_var_ = eskf_param_.bias_acce_m_var;
    options.vision_xy_noise_ = eskf_param_.vision_xy_noise;
    options.vision_z_noise_ = eskf_param_.vision_z_noise;
    options.vision_roll_pitch_noise_ = eskf_param_.vision_roll_pitch_noise;
    options.vision_yaw_noise_ = eskf_param_.vision_yaw_noise;
    options.update_bias_gyro_ = eskf_param_.update_bias_gyro;
    options.update_bias_acce_ = eskf_param_.update_bias_acce;

    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -eskf_param_.gravity);

    eskf_.SetInitialConditions(options, eskf_param_.gyro_bias, eskf_param_.acce_bias, gravity);
}

void LandingTargetPose::Init()
{
    imu_timeout_ = 0.5;
    tf_timeout_ = 0.5;
    imu_init_flag_ = false;
    imu0_init_flag_ = false;
    // tf_init_flag_ = false;
    get_new_landing_target_ = false;
    get_new_landing_target_time_ = ros::Time(0);
    eskf_init_flag_ = false;

    //lc add: 初始化视觉监测状态
    last_vision_time_ = 0.0;
    vision_timeout_ = 1.0;  // 1秒超时
    vision_valid_ = false;
    eskf_enabled_ = false;

    // eskf_param_.eskf_init = false;

    // R_em_ = Eigen::Matrix3d::Identity();

    // R_sc_ = frame_param_.body_to_camera_q.toRotationMatrix();
    // R_mt_ = frame_param_.uav0_to_tag_q.toRotationMatrix();
    // R_ml_ = frame_param_.uav0_to_landing_q.toRotationMatrix();

    // ESKFD::Options options;
    // options.imu_dt_ = 1.0 / eskf_param_.eskf_hz;
    // options.gyro_var_ = eskf_param_.gyro_var;
    // options.acce_var_ = eskf_param_.acce_var;
    // options.bias_gyro_var_ = eskf_param_.bias_gyro_var;
    // options.bias_acce_var_ = eskf_param_.bias_acce_var;
    // options.vision_xy_noise_ = eskf_param_.vision_xy_noise;
    // options.vision_z_noise_ = eskf_param_.vision_z_noise;
    // options.vision_roll_pitch_noise_ = eskf_param_.vision_roll_pitch_noise;
    // options.vision_yaw_noise_ = eskf_param_.vision_yaw_noise;
    // options.update_bias_gyro_ = eskf_param_.update_bias_gyro;
    // options.update_bias_acce_ = eskf_param_.update_bias_acce;

    // Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -eskf_param_.gravity);

    // eskf_.SetInitialConditions(options, eskf_param_.gyro_bias, eskf_param_.acce_bias, gravity);

    SetEskfParam();

    eskf_.SetConstant(frame_param_.body_to_camera_p, frame_param_.body_to_camera_q, frame_param_.uav0_to_tag_p, frame_param_.uav0_to_tag_q);
}

void LandingTargetPose::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ = *msg;
    imu_init_flag_ = true;
}

void LandingTargetPose::Imu0Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu0_ = *msg;
    imu0_init_flag_ = true;
}

void LandingTargetPose::LocalPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_local_pos_ = *msg;
}

void LandingTargetPose::TagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->detections.size() == 0)
    {
        return;
    }

    for (const auto &detection : msg->detections)
    {
        if (detection.id[0] == 0 && detection.id[1] == 1)
        {
            if (std::fabs(detection.pose.pose.pose.position.z) > tag_param_.tag_valid_distance)
            {
                return;
            }
            tag_pose_.header = detection.pose.header;
            tag_pose_.pose = detection.pose.pose.pose;
            get_new_landing_target_ = true;
            get_new_landing_target_time_ = ros::Time::now();

            //lc add: 标记视觉数据有效
            last_vision_time_ = ros::Time::now().toSec();
            vision_valid_ = true;
        
            //lc add: 视觉恢复时重新激活ESKF
            if (!eskf_enabled_) {
                ROS_INFO("Vision recovered, activating ESKF");
                eskf_enabled_ = true;
            }

            break;
        }
    }
}

// void LandingTargetPose::TagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
// {
//     static int count;

//     if (msg->detections.size() == 0)
//     {
//         return;
//     }

//     count++;

//     if(count < 16)
//     {
//         for (const auto &detection : msg->detections)
//         {
//             if (detection.id[0] == 0 && detection.id[1] == 1)
//             {
//                 if (std::fabs(detection.pose.pose.pose.position.z) > tag_param_.tag_valid_distance)
//                 {
//                     return;
//                 }
//                 tag_pose_.header = detection.pose.header;
//                 tag_pose_.pose = detection.pose.pose.pose;
//                 get_new_landing_target_ = true;
//                 get_new_landing_target_time_ = ros::Time::now();

//                 //lc add: 标记视觉数据有效
//                 last_vision_time_ = ros::Time::now().toSec();
//                 vision_valid_ = true;
            
//                 //lc add: 视觉恢复时重新激活ESKF
//                 if (!eskf_enabled_) {
//                     ROS_INFO("Vision recovered, activating ESKF");
//                     eskf_enabled_ = true;
//                 }

//                 break;
//             }
//         }
//     }
//     else{
//         if(count > 30)
//             count = 0;  
//         return;
//     }    
// }

void LandingTargetPose::GazeboTrueCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "iris0")
        {
            q_em_ = Eigen::Quaterniond(msg->pose[i].orientation.w, msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z);
        }
        else if (msg->name[i] == "iris1")
        {
            uav_local_pos_.pose = msg->pose[i];
            uav_local_pos_.pose.position.x -= 1.0;
        }
    }
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

    if ((x_d > 0.1 || y_d > 0.1 || z_d > 0.1 || q_x_d > 0.1 || q_y_d > 0.1 || q_z_d > 0.1 || q_w_d > 0.1) && eskf_outlier_reject_count < 3)
    {
        eskf_outlier_reject_count++;
        return false;
    }

    last_tag_pose_ = tag_pose_;

    eskf_outlier_reject_count = 0;

    return true;
}

// bool LandingTargetPose::GetTagPose()
// {
//     try
//     {
//         camera_to_tag_tf_ = tf_buffer_.lookupTransform(tag_param_.tag_frame_id, frame_param_.camera_frame_id, ros::Time(0));
//         // if (camera_to_tag_tf_.header.stamp < ros::Time::now() - ros::Duration(tf_timeout_))
//         // {
//         //     // ROS_WARN("tf timeout");
//         //     return false;
//         // }
//         if (std::fabs(camera_to_tag_tf_.transform.translation.z) > tag_param_.tag_valid_distance)
//         {
//             // ROS_WARN("tag too far");
//             return false;
//         }
//     }
//     catch (tf2::TransformException &ex)
//     {
//         // ROS_WARN("%s", ex.what());
//         return false;
//     }

//     if (camera_to_tag_tf_.header.stamp.toSec() > tag_pose_.header.stamp.toSec() + 0.001)
//     {
//         get_new_landing_target_ = true;
//         // 将camera_to_tag_tf_ 转换为 geometry_msgs::PoseStamped
//         tag_pose_.header = camera_to_tag_tf_.header;
//         tag_pose_.pose.position.x = camera_to_tag_tf_.transform.translation.x;
//         tag_pose_.pose.position.y = camera_to_tag_tf_.transform.translation.y;
//         tag_pose_.pose.position.z = camera_to_tag_tf_.transform.translation.z;
//         tag_pose_.pose.orientation = camera_to_tag_tf_.transform.rotation;

//         landing_target_pose_pub_.publish(tag_pose_);
//     }

//     return true;
// }

void LandingTargetPose::EskfTimerCallback(const ros::TimerEvent &event)
{
    // ================= 新增视觉超时检测 =================
    const double current_time = ros::Time::now().toSec();
    
    // 检查视觉超时（仅在已初始化后生效）
    if (eskf_init_flag_ && (current_time - last_vision_time_ > vision_timeout_)) 
    {
        if (eskf_enabled_) {
            ROS_ERROR_STREAM("[ESKF] Vision timeout! Last update: " 
                            << current_time - last_vision_time_ << "s ago");
            eskf_enabled_ = false;
        }
        vision_valid_ = false;
    }

    if (!imu_init_flag_) {
        return;
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
        
        //lc add: 更新最后视觉时间戳
        last_vision_time_ = current_time;
        vision_valid_ = true;
    }

    // ================= ESKF初始化逻辑 =================
    if (!eskf_init_flag_ && get_new_landing_target_) {
        eskf_.Reset();

        //lc modified: 初始化时激活ESKF
        eskf_enabled_ = true;
        get_new_landing_target_ = false;
        last_tag_pose_ = tag_pose_;
        eskf_.ObserveVision(tag_pose_);

        eskf_init_time_ = current_time;
        eskf_init_flag_ = true;
        
        ROS_INFO("[ESKF] Initialized with first vision data");
    }

    // ================= 预测逻辑控制 =================
    if (eskf_enabled_ && eskf_init_flag_) { 
        try {
            eskf_.Predict(imu_, imu0_);
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("ESKF prediction failed: " << e.what());
        }
    }

    // ================= 观测逻辑控制 =================
    if (get_new_landing_target_) {
        get_new_landing_target_ = false;
        
        //lc modified: 仅在使能状态执行观测更新
        if (eskf_enabled_ && IsTagPoseValid()) {
            try {
                eskf_.ObserveVision(tag_pose_);
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("ESKF observation failed: " << e.what());
            }
        }
    }

    // ================= 数据发布控制 =================
    if (eskf_enabled_ && eskf_init_flag_) {
        // 正常发布数据
        nominal_state_ = eskf_.GetNominalSE3();
        CalculateLandingTargetPose();
        landing_target_pose_pub_.publish(landing_target_pose_);
    } else if (eskf_init_flag_) {  // 已初始化但被禁用
        static ros::Time last_warn_time = ros::Time(0);
        if ((ros::Time::now() - last_warn_time).toSec() > 1.0) {
            ROS_WARN_THROTTLE(1.0, "[ESKF] Output disabled due to vision timeout");
            last_warn_time = ros::Time::now();
        }
    }
}


bool LandingTargetPose::IsLandingTargetDetected()
{
    static bool last_get_landing_target = false;
    if ((ros::Time::now() - get_new_landing_target_time_).toSec() > 1)
    {
        if (last_get_landing_target)
        {
            ROS_WARN("landing target lost");
        }
        last_get_landing_target = false;
        eskf_init_flag_ = false;
        return false;
    }
    last_get_landing_target = true;
    return true;
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
