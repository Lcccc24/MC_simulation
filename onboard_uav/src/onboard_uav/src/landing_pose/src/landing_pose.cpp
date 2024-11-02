#include "landing_pose/landing_pose.hpp"

LandingPose::LandingPose(ros::NodeHandle &nh) : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{

    // 读取参数
    nh_.param("ekf_param/ekf_hz", ekf_param_.ekf_hz, 100);
    nh_.param("ekf_param/accel_noise", ekf_param_.accel_noise, 2.5);
    nh_.param("ekf_param/xy_pos_var", ekf_param_.xy_pos_var, 0.1);
    nh_.param("ekf_param/z_pos_var", ekf_param_.z_pos_var, 0.2);
    nh_.param("ekf_param/vel_var", ekf_param_.vel_var, 0.25);

    nh_.param("docking/tag_frame_id", tag_param_.tag_frame_id, std::string("tag36h11"));
    nh_.param("docking/tag_offset_x", tag_param_.tag_offset_x, 0.0);
    nh_.param("docking/tag_offset_y", tag_param_.tag_offset_y, 0.250);
    nh_.param("docking/tag_offset_z", tag_param_.tag_offset_z, 0.02);
    nh_.param("docking/tag_offset_yaw", tag_param_.tag_offset_yaw, 0.0);
    nh_.param("docking/tag_valid_distance", tag_param_.tag_valid_distance, 3.0);

    nh_.param("msg_timeout/uav_vel", uav_vel_timeout_, 0.5);
    nh_.param("msg_timeout/tf", tf_timeout_, 0.5);

    uav_vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_body", 1, &LandingPose::UavVelCallback, this);
    landing_target_pose_raw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_pose_raw", 1);
    landing_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/landing_target_pose", 1);

    ekf_timer_ = nh_.createTimer(ros::Duration(1.0 / ekf_param_.ekf_hz), &LandingPose::EkfTimerCallback, this);

    Init();
}

LandingPose::~LandingPose()
{
}

void LandingPose::Init()
{
    // 根据tag_param_计算降落目标在标签坐标系下的位姿
    tf2::Quaternion q;
    q.setRPY(0, 0, tag_param_.tag_offset_yaw);
    landing_target_pose_in_tag_frame_.header.frame_id = tag_param_.tag_frame_id;
    landing_target_pose_in_tag_frame_.pose.position.x = tag_param_.tag_offset_x;
    landing_target_pose_in_tag_frame_.pose.position.y = tag_param_.tag_offset_y;
    landing_target_pose_in_tag_frame_.pose.position.z = tag_param_.tag_offset_z;
    landing_target_pose_in_tag_frame_.pose.orientation = tf2::toMsg(q);

    uav_vel_.header.stamp = ros::Time(0);
    landing_target_body_pose_raw_.header.stamp = ros::Time(0);

    ekf_dt_ = 1.0 / ekf_param_.ekf_hz;
    ekf_init_time_ = 0.0;
    ekf_last_time_ = 0.0;
    ekf_init_flag_ = false;

    get_new_landing_target_ = false;
}

/**
 * @brief 订阅uav的速度
 */
void LandingPose::UavVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    while (uav_vel_lock_.test_and_set(std::memory_order_acquire))
        ;
    uav_vel_ = *msg;
    uav_vel_lock_.clear(std::memory_order_release);
}

/**
 * @brief 获取uav相对于标签的速度
 * @param uav_vel uav相对于标签的速度
 * @return 是否获取成功
 */
bool LandingPose::GetTagVel(Eigen::Vector3d &tag_vel)
{
    while (uav_vel_lock_.test_and_set(std::memory_order_acquire))
        ;
    if (ros::Time::now() - uav_vel_.header.stamp > ros::Duration(uav_vel_timeout_))
    {
        uav_vel_lock_.clear(std::memory_order_release);
        return false;
    }

    tag_vel.x() = -uav_vel_.twist.linear.x;
    tag_vel.y() = -uav_vel_.twist.linear.y;
    tag_vel.z() = -uav_vel_.twist.linear.z;
    uav_vel_lock_.clear(std::memory_order_release);
    return true;
}

/**
 * @brief 监听tf，获取body坐标系下的降落目标位姿
 * @return 是否获取成功
 */
void LandingPose::GetTagPose()
{
    ros::Time time_now = ros::Time::now();

    // 监听tf，当大标签的坐标变换不存在时，尝试获取小标签的坐标变换
    try
    {
        body_tag_tf_ = tf_buffer_.lookupTransform("base_link", tag_param_.tag_frame_id, ros::Time(0));
        if (body_tag_tf_.header.stamp < time_now - ros::Duration(tf_timeout_))
        {
            // try
            // {
            //     body_tag_tf_ = tf_buffer_.lookupTransform("base_link", tag_param_.small_tag_frame_id, ros::Time(0));
            //     if (body_tag_tf_.header.stamp < time_now - ros::Duration(tf_timeout_))
            //     {
            //         // ROS_WARN("body_tag_tf_ is too old");
            //         return;
            //     }
            //     else
            //     {
            //         if (body_tag_tf_.header.stamp.toSec() > landing_target_body_pose_raw_.header.stamp.toSec() + 0.001)
            //         {
            //             get_new_landing_target_ = true;
            //         }
            //         tf2::doTransform(landing_target_pose_small_tag_, landing_target_body_pose_raw_, body_tag_tf_);
            //         // 当z轴距离大于有效检测距离时，认为小标签位姿不可靠
            //         if (-landing_target_body_pose_raw_.pose.position.z > tag_param_.small_tag_valid_distance)
            //         {
            //             get_new_landing_target_ = false;
            //         }
            //     }
            // }
            // catch (tf2::TransformException &ex)
            // {
            //     // ROS_WARN("GetTagPose failed: %s", ex.what());
            //     return;
            // }
        }
        else
        {
            if (body_tag_tf_.header.stamp.toSec() > landing_target_body_pose_raw_.header.stamp.toSec() + 0.001)
            {
                get_new_landing_target_ = true;
            }
            tf2::doTransform(landing_target_pose_in_tag_frame_, landing_target_body_pose_raw_, body_tag_tf_);
            // 当z轴距离大于有效检测距离时，认为大标签位姿不可靠
            if (-landing_target_body_pose_raw_.pose.position.z > tag_param_.tag_valid_distance)
            {
                get_new_landing_target_ = false;
            }
        }
    }
    catch (tf2::TransformException &ex)
    {
        // ROS_WARN("GetTagPose failed: %s", ex.what());
        return;
    }
    return;
}

/**
 * @brief EKF定时回调函数
 * @param event 定时器事件
 */
void LandingPose::EkfTimerCallback(const ros::TimerEvent &event)
{
    GetTagPose();

    Eigen::Vector3d tag_vel;
    if (!GetTagVel(tag_vel))
    {
        return;
    }
    static Eigen::Vector3d last_tag_vel = Eigen::Vector3d::Zero();
    const Eigen::Vector3d delta_tag_vel = tag_vel - last_tag_vel;
    last_tag_vel = tag_vel;

    static ros::Time last_ekf_time = ros::Time::now();

    // 如果EKF已经初始化或者检测到降落目标时，进行预测
    if (ekf_init_flag_ || IsLandingTargetDetected())
    {
        ekf_dt_ = (ros::Time::now() - last_ekf_time).toSec();
        last_ekf_time = ros::Time::now();

        x_ekf_.predict(ekf_dt_, delta_tag_vel.x(), ekf_param_.accel_noise * ekf_dt_);
        y_ekf_.predict(ekf_dt_, delta_tag_vel.y(), ekf_param_.accel_noise * ekf_dt_);
        z_ekf_.predict(ekf_dt_, delta_tag_vel.z(), ekf_param_.accel_noise * ekf_dt_);
    }

    // 如果有新的测量值可用，则进行更新
    if (get_new_landing_target_)
    {
        get_new_landing_target_ = false;
        // xy方向的位置测量噪声方差
        // double xy_pos_var = ekf_param_.xy_pos_var * std::max(1.0, std::fabs(landing_target_body_pose_raw_.pose.position.z) - 2.0);
        // z方向的位置测量噪声方差
        double z_dist_square = landing_target_body_pose_raw_.pose.position.z * landing_target_body_pose_raw_.pose.position.z;
        // double z_pos_var = ekf_param_.z_pos_var * std::max(0.1, z_dist_square + uav_vel.norm());

        double xy_pos_var = ekf_param_.xy_pos_var * z_dist_square;
        double z_pos_var = ekf_param_.z_pos_var * z_dist_square;

        // 速度测量噪声方差
        double vel_var = ekf_param_.vel_var;

        //  如果EKF未初始化，则初始化
        if (!ekf_init_flag_)
        {
            x_ekf_.init(landing_target_body_pose_raw_.pose.position.x, xy_pos_var, tag_vel.x(), vel_var);
            y_ekf_.init(landing_target_body_pose_raw_.pose.position.y, xy_pos_var, tag_vel.y(), vel_var);
            z_ekf_.init(landing_target_body_pose_raw_.pose.position.z, z_pos_var, tag_vel.z(), vel_var);
            ekf_init_time_ = ros::Time::now().toSec();
            ekf_last_time_ = ekf_init_time_;
            ekf_init_flag_ = true;
        }
        else
        {
            static int ekf_outlier_reject_count = 0;
            double nis_x = x_ekf_.getPosNIS(landing_target_body_pose_raw_.pose.position.x, xy_pos_var);
            double nis_y = y_ekf_.getPosNIS(landing_target_body_pose_raw_.pose.position.y, xy_pos_var);
            double nis_z = z_ekf_.getPosNIS(landing_target_body_pose_raw_.pose.position.z, z_pos_var);
            if (std::max(std::max(nis_x, nis_y), nis_z) < 3.0 || ekf_outlier_reject_count >= 3)
            {
                ekf_outlier_reject_count = 0;
                x_ekf_.fusePos(landing_target_body_pose_raw_.pose.position.x, xy_pos_var);
                y_ekf_.fusePos(landing_target_body_pose_raw_.pose.position.y, xy_pos_var);
                z_ekf_.fusePos(landing_target_body_pose_raw_.pose.position.z, z_pos_var);
                ekf_last_time_ = ros::Time::now().toSec();
            }
            else
            {
                ++ekf_outlier_reject_count;
            }
        }
    }

    // 检查是否初始化成功
    if (ekf_init_flag_ && !IsLandingTargetDetected())
    {
        // 如果传感器数据更新频率过低，则认为初始化失败
        if (ros::Time::now().toSec() - ekf_last_time_ > 0.5)
        {
            ekf_init_flag_ = false;
            ROS_WARN("EKF init failed");
        }
        // 滤波2s后认为初始化成功
        else if (ros::Time::now().toSec() - ekf_init_time_ > 2.0)
        {

            ROS_INFO("EKF init success");
        }
    }

    // 输出滤波结果
    if (IsLandingTargetDetected())
    {
        landing_target_body_pose_.header = landing_target_body_pose_raw_.header;
        landing_target_body_pose_.pose.position.x = x_ekf_.getPos();
        landing_target_body_pose_.pose.position.y = y_ekf_.getPos();
        landing_target_body_pose_.pose.position.z = z_ekf_.getPos();
        landing_target_body_pose_.pose.orientation = landing_target_body_pose_raw_.pose.orientation;

        // 转化为local ENU坐标系下的位姿后发布
        if (TransformLandingTargetPose())
        {
            landing_target_pose_raw_pub_.publish(landing_target_local_pose_raw_);
            landing_target_pose_pub_.publish(landing_target_local_pose_);
        }
    }

    // 获取定时器的周期
    // ekf_dt_ = event.current_real.toSec() - event.last_real.toSec();
}

/**
 * @brief 判断降落目标是否检测到
 * @return 是否检测到
 */

bool LandingPose::IsLandingTargetDetected()
{
    // 如果传感器数据超时2s，则认为降落目标丢失
    static bool last_get_landing_target = false;
    if (ros::Time::now().toSec() - ekf_last_time_ > 0.5)
    {
        if (last_get_landing_target)
        {
            ROS_WARN("Landing target lost!");
        }
        last_get_landing_target = false;
        ekf_init_flag_ = false;
        return false;
    }
    last_get_landing_target = true;
    return true;
}

/**
 * @brief 将降落目标位姿从body转换到local ENU坐标系下
 * @return 是否转化成功
 */
bool LandingPose::TransformLandingTargetPose()
{
    // 监听tf，将降落目标位姿从body坐标系转换到local坐标系下
    try
    {
        tf_buffer_.transform(landing_target_body_pose_raw_, landing_target_local_pose_raw_, "map");
        tf_buffer_.transform(landing_target_body_pose_, landing_target_local_pose_, "map");
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transform landing target pose failed: %s", ex.what());
        return false;
    }
    return true;
}