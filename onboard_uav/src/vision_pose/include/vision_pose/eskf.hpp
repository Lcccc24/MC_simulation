//
// Created by xiang on 2021/11/11.
//

#pragma once

#include "vision_pose/eigen_types.hpp"
// #include "common/gnss.h"
// #include "common/imu.h"
// #include "vision_pose/math_utils.hpp"
#include "vision_pose/nav_state.hpp"
// #include "common/odom.h"

// #include <glog/logging.h>
// #include <iomanip>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

// namespace sad
// {

/**
 * 书本第3章介绍的误差卡尔曼滤波器
 * 可以指定观测GNSS的读数，GNSS应该事先转换到车体坐标系
 *
 * 本书使用18维的ESKF，标量类型可以由S指定，默认取double
 * 变量顺序：p, v, R, bg, ba, grav，与书本对应
 * @tparam S    状态变量的精度，取float或double
 */
template <typename S = double>
class ESKF
{
public:
    /// 类型定义
    using SO3 = Sophus::SO3<S>;                    // 旋转变量类型
    using VecT = Eigen::Matrix<S, 3, 1>;           // 向量类型
    using Vec18T = Eigen::Matrix<S, 18, 1>;        // 18维向量类型
    using Mat3T = Eigen::Matrix<S, 3, 3>;          // 3x3矩阵类型
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>; // 运动噪声类型
    using VisionNoiseT = Eigen::Matrix<S, 6, 6>;   // 视觉噪声类型
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;     // 里程计噪声类型
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;     // GNSS噪声类型
    using Mat18T = Eigen::Matrix<S, 18, 18>;       // 18维方差类型
    using NavStateT = NavState<S>;                 // 整体名义状态变量类型

    struct Options
    {
        Options() = default;

        /// IMU 测量与零偏参数
        double imu_dt_ = 0.01; // IMU测量间隔
        // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
        double gyro_var_ = 1e-5;        // 陀螺测量标准差
        double acce_var_ = 1e-2;        // 加计测量标准差
        double bias_gyro_var_ = 1e-6;   // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;   // 加计零偏游走标准差
        double bias_acce_m_var_ = 1e-4; // 母机加计零偏游走标准差

        /// 视觉观测参数
        // xy位置噪声
        double vision_xy_noise_ = 0.2;
        // z位置噪声
        double vision_z_noise_ = 0.3;
        // 横滚角、俯仰角噪声
        double vision_roll_pitch_noise_ = 0.35;
        // 偏航角噪声
        double vision_yaw_noise_ = 0.05;

        /// 其他配置
        bool update_bias_gyro_ = true; // 是否更新陀螺bias
        bool update_bias_acce_ = true; // 是否更新加计bias
    };

    /**
     * 初始零偏取零
     */
    ESKF(Options option = Options()) : options_(option) { BuildProcessNoise(option); }

    /**
     * 设置初始条件
     * @param options 噪声项配置
     * @param init_bg 初始零偏 陀螺
     * @param init_ba 初始零偏 加计
     * @param gravity 重力
     */
    void SetInitialConditions(Options options, const VecT &init_bg, const VecT &init_ba,
                              const VecT &gravity = VecT(0, 0, -9.8))
    {
        BuildProcessNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        // g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;
    }

    /// 使用IMU递推
    bool Predict(const sensor_msgs::Imu &imu, const sensor_msgs::Imu &imu_m);

    /// 使用视觉观测
    bool ObserveVision(const geometry_msgs::PoseStamped &pose);

    /**
     * 使用SE3进行观测
     * @param pose  观测位姿
     * @param trans_noise 平移噪声
     * @param ang_noise   角度噪声
     * @return
     */
    bool ObserveSE3(const SE3 &pose);

    /// accessors
    /// 获取全量状态
    NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

    /// 获取SE3 状态
    SE3 GetNominalSE3() const { return SE3(R_, p_); }

    // SE3 GetPsmRct() const { return SE3(R_ct_, p_); }

    /// 设置状态X
    void SetX(const NavStated &x, const Vec3d &grav)
    {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        // g_ = grav;
    }

    /// 设置协方差
    void SetCov(const Mat18T &cov) { cov_ = cov; }

    // 设置坐标系常量
    void SetConstant(const Vec3d &p_sc, const Eigen::Quaterniond &q_sc, const Vec3d &p_mt, const Eigen::Quaterniond &q_mt)
    {
        p_sc_ = p_sc;
        R_sc_ = SO3(q_sc);
        p_mt_ = p_mt;
        R_mt_ = SO3(q_mt);
    }

    /// 获取重力
    // Vec3d GetGravity() const
    // {
    //     return g_;
    // }

    void Reset()
    {
        // R_ = SO3(q_es);
        first_vision_ = true;
    }

private:
    void BuildProcessNoise(const Options &options)
    {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;
        double eam = options.bias_acce_m_var_;

        double ev2 = ev * ev;
        double et2 = et * et;
        double eg2 = eg * eg;
        double ea2 = ea * ea;
        double eam2 = eam * eam;

        // 设置过程噪声 TODO
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, ea2, ea2, ea2, eam2, eam2, eam2, eg2, eg2, eg2;
    }

    /// 更新名义状态变量，重置error state
    void UpdateAndReset()
    {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (options_.update_bias_acce_)
        {
            ba_ += dx_.template block<3, 1>(9, 0);
            ba_m_ += dx_.template block<3, 1>(12, 0);
        }
        if (options_.update_bias_gyro_)
        {
            bg_ += dx_.template block<3, 1>(15, 0);
        }

        // g_ += dx_.template block<3, 1>(15, 0);

        ProjectCov();
        dx_.setZero();
    }

    /// 对P阵进行投影，参考式(3.63)
    void ProjectCov()
    {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

    // 计算观测噪声
    void CalcVisionNoise(double z)
    {
        double z2 = z * z + 0.1;
        double vxy2 = options_.vision_xy_noise_ * options_.vision_xy_noise_ * (std::fabs(z) + 0.3);
        double vz2 = options_.vision_z_noise_ * options_.vision_z_noise_ * (std::fabs(z) + 0.3);
        double vrp2 = options_.vision_roll_pitch_noise_ * options_.vision_roll_pitch_noise_;
        double vy2 = options_.vision_yaw_noise_ * options_.vision_yaw_noise_;
        vision_noise_.diagonal() << vxy2, vxy2, vz2, vrp2, vrp2, vy2;
    }

    // void CalcVisionPose()
    // {
    //     p_ct_ = R_sc_.matrix().transpose() * R_.matrix().transpose() * (p_ + R_em_.matrix() * p_mt_) - R_sc_.matrix().transpose() * p_sc_;
    //     R_ct_ = R_sc_.inverse() * R_.inverse() * R_em_ * R_mt_;
    // }

    /// 成员变量

    // 坐标系常量
    VecT p_sc_, p_mt_, p_ml_;
    SO3 R_sc_, R_mt_, R_ml_;

    // // 母机姿态
    // SO3 R_em_;

    // // 视觉观测量
    // VecT p_ct_;
    // SO3 R_ct_;

    double current_time_ = 0.0; // 当前时间

    /// 名义状态
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    SO3 R_;
    VecT ba_ = VecT::Zero();
    VecT ba_m_ = VecT::Zero();
    VecT bg_ = VecT::Zero();

    /// 误差状态
    Vec18T dx_ = Vec18T::Zero();

    /// 协方差阵
    Mat18T cov_ = Mat18T::Identity();

    /// 噪声阵
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    VisionNoiseT vision_noise_ = VisionNoiseT::Zero();

    /// 标志位
    bool first_vision_ = true; // 是否为第一个视觉数据

    /// 配置项
    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
bool ESKF<S>::Predict(const sensor_msgs::Imu &imu, const sensor_msgs::Imu &imu_m)
{
    assert(imu.header.stamp.toSec() >= current_time_);

    // R_em_ = SO3(q_em);

    double dt = imu.header.stamp.toSec() - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0)
    {
        // 时间间隔不对，可能是第一个IMU数据，没有历史信息
        std::cout << "IMU timeout, dt: " << dt << std::endl;
        current_time_ = imu.header.stamp.toSec();
        return false;
    }

    Vec3d gyro = Vec3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
    Vec3d acce = Vec3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    // Vec3d gyro_m = Vec3d(imu_m.angular_velocity.x, imu_m.angular_velocity.y, imu_m.angular_velocity.z);
    Vec3d acce_m = Vec3d(imu_m.linear_acceleration.x, imu_m.linear_acceleration.y, imu_m.linear_acceleration.z);

    // nominal state 递推
    VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (acce - ba_)) * dt * dt - 0.5 * (acce_m - ba_m_) * dt * dt;
    VecT new_v = v_ + R_ * (acce - ba_) * dt - (acce_m - ba_m_) * dt;
    SO3 new_R = R_ * SO3::exp((gyro - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;
    // 其余状态维度不变

    // error state 递推
    // 计算运动过程雅可比矩阵 F，见(3.47)
    // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
    // 注意bg和ba的顺序
    Mat18T F = Mat18T::Identity();                                           // 主对角线
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                   // p 对 v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(acce - ba_) * dt; // v对theta
    F.template block<3, 3>(3, 9) = -R_.matrix() * dt;                        // v 对 ba
    F.template block<3, 3>(3, 12) = Mat3T::Identity() * dt;                  // v 对 ba_m
    F.template block<3, 3>(6, 6) = SO3::exp(-(gyro - bg_) * dt).matrix();    // theta 对 theta
    F.template block<3, 3>(6, 15) = -Mat3T::Identity() * dt;                 // theta 对 bg

    // mean and cov prediction
    dx_ = F * dx_; // 这行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = imu.header.stamp.toSec();
    // CalcVisionPose();
    return true;
}

template <typename S>
bool ESKF<S>::ObserveVision(const geometry_msgs::PoseStamped &pose)
{
    // 判断时间戳
    // assert(pose.header.stamp.toSec() >= current_time_);

    // R_em_ = SO3(q_em);

    VecT p_ct(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // SO3 R_ct(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    // SO3 R_ct = SO3::Quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    SO3 R_ct(q);

    if (first_vision_)
    {
        // 应该初始化名义状态
        // p_ct_ = p_ct;
        // R_ct_ = R_ct;

        p_ = p_mt_ - R_ * (p_sc_ + R_sc_.matrix() * p_ct);
        R_ = R_mt_ * R_ct.inverse() * R_sc_.inverse();

        first_vision_ = false;
        // current_time_ = pose.header.stamp.toSec();

        // CalcVisionNoise(p_ct(2));

        return true;
    }

    CalcVisionNoise(p_ct(2));

    ObserveSE3(SE3(R_ct, p_ct));
    // current_time_ = pose.header.stamp.toSec();

    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3 &pose)
{
    /// 既有旋转，也有平移
    /// 观测状态变量中的p, R，H为6x18，其余为零
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    Mat3T h11 = -R_sc_.matrix().transpose() * SO3::exp(-dx_.template block<3, 1>(6, 0)).matrix() * R_.matrix().transpose();
    Mat3T h13 = R_sc_.matrix().transpose() * SO3::hat(R_.matrix().transpose() * (p_mt_ - p_ - dx_.template block<3, 1>(0, 0)));
    Mat3T h23 = -R_mt_.matrix().transpose() * R_.matrix();



    H.template block<3, 3>(0, 0) = h11; // p 对 dp
    H.template block<3, 3>(0, 6) = h13; // p 对 dtheta
    H.template block<3, 3>(3, 6) = h23; // R 对 dtheta

    // 卡尔曼增益和更新过程
    Mat6d V = vision_noise_;
    Eigen::Matrix<S, 18, 6>
        K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    // 根据预测值计算观测值
    Vec3d p_ct_hat = R_sc_.matrix().transpose() * R_.matrix().transpose() * (p_mt_ - p_) - R_sc_.matrix().transpose() * p_sc_;
    SO3 R_ct_hat = R_sc_.inverse() * R_.inverse() * R_mt_;

    // 更新x和cov
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_ct_hat);         // 平移部分
    innov.template tail<3>() = (R_ct_hat.inverse() * pose.so3()).log(); // 旋转部分(3.67)

    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}

// } // namespace sad
