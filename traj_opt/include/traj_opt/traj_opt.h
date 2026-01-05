#pragma once
#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <vis_utils/vis_utils.hpp>

#include "minco.hpp"

namespace traj_opt
{

    struct LandingParams {
        bool is_landing = false;
        double land_x = 0.0;
        double land_y = 0.0;
        double land_z = 0.0;
        double uwb_dist = 0.0;
    };

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
        double rho_D_, rhoOmega_, rho_LV_;
        double LV_max_, LV_min_;
        double emergency_stop_dist_, safe_aera_radius_;
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
            double cost_p_ = 0.0;
            double cost_v_ = 0.0;
            double cost_a_ = 0.0;
            double cost_j_ = 0.0;
            double cost_d_ = 0.0;
            double cost_lv_ = 0.0;
            double cost_omega_ = 0.0;
        };

        violate_cost violate_cost_;

        double land_target_x_ = 0.0;
        double land_target_y_ = 0.0;
        double land_target_z_ = 0.0;
        double uwb_dist_ = 0.0;
        

    public:
        TrajOpt(ros::NodeHandle &nh);
        ~TrajOpt() {}

        void setLandingParams(const LandingParams &lp);
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

        bool StrongWindAreaGradCostP(const Eigen::Vector3d &p, Eigen::Vector3d &gradp, double &costp);

        bool feasibilityGradCostV(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costv);

        bool feasibilityGradCostA(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa);

        bool feasibilityGradCostJ(const Eigen::Vector3d &j, Eigen::Vector3d &gradj, double &costj);

        bool feasibilityGradCostOmega(const Eigen::Vector3d &a, const Eigen::Vector3d &j, Eigen::Vector3d &grada, Eigen::Vector3d &gradj, double &cost);

        bool EmerDistGradCostD(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costd);

        bool LandSmoothGradCost(const Eigen::Vector3d &p, const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costlv);

        double computeAllowedVmaxLV(double delta_z);

        double computePenaltyWeightLV(double delta_z);       
    };

} // namespace traj_opt