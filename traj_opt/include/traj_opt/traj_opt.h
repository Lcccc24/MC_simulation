#pragma once
#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <vis_utils/vis_utils.hpp>

#include "minco.hpp"

namespace traj_opt
{

    class TrajOpt
    {
    public:
        ros::NodeHandle nh_;
        std::shared_ptr<vis_utils::VisUtils> visPtr_;
        bool pause_debug_ = false;
        bool is_landing_ = false;
        // # pieces and # key points
        int N_, K_, dim_t_, dim_p_;
        // weight for time regularization term
        double rhoT_;
        // collision avoiding and dynamics paramters
        double vmax_, amax_, jmax_;
        double rhoP_, rhoV_, rhoA_, rhoJ_;
        double rhoOmega_;
        // SE3 dynamic limitation parameters
        double omega_max_, omega_yaw_max_;
        // MINCO Optimizer
        minco::MINCO_S4 mincoOpt_;
        Eigen::MatrixXd initS_;
        // duration of each piece of the trajectory
        Eigen::VectorXd t_;
        double *x_;

        std::vector<Eigen::Vector3d> tracking_ps_;
        std::vector<Eigen::Vector3d> tracking_visible_ps_;
        std::vector<double> tracking_thetas_;

        struct violate_cost {
            double cost_v_ = 0.0;
            double cost_a_ = 0.0;
            double cost_j_ = 0.0;
            double cost_omega_ = 0.0;
        };

        violate_cost violate_cost_;

    public:
        TrajOpt(ros::NodeHandle &nh);
        ~TrajOpt() {}

        void setLandingParams(const bool is_landing);
        int optimize(const double &delta = 1e-4);
        bool generate_traj(const Eigen::MatrixXd &iniState,
                           const Eigen::Vector3d &car_p,
                           const Eigen::Vector3d &car_v,
                           const Eigen::Quaterniond &land_q,
                           const int &N,
                           Trajectory &traj,
                           const double &t_replan = -1.0);

        //lc add
        bool trans_bvp_traj(Trajectory &traj);

        void addTimeIntPenalty(double &cost);

        bool feasibilityGradCostV(const Eigen::Vector3d &v,
                         Eigen::Vector3d &gradv,
                         double &costv);

        bool feasibilityGradCostA(const Eigen::Vector3d &a,
                         Eigen::Vector3d &grada,
                         double &costa);

        bool feasibilityGradCostJ(const Eigen::Vector3d &j,
                         Eigen::Vector3d &gradj,
                         double &costj);

        bool grad_cost_omega(const Eigen::Vector3d &a,
                             const Eigen::Vector3d &j,
                             Eigen::Vector3d &grada,
                             Eigen::Vector3d &gradj,
                             double &cost);
    };

} // namespace traj_opt