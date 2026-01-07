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
        Eigen::Vector3d m_uav_pos = Eigen::Vector3d::Zero();
    };

    class TrajOpt
    {
    public:
        ros::NodeHandle nh_;
        std::shared_ptr<vis_utils::VisUtils> visPtr_;
        bool is_landing_ = false;
        int N_, K_, dim_t_, dim_p_;
        double vmax_, amax_, jmax_;
        double rhoT_;
        double rhoP_, rhoV_, rhoA_, rhoJ_;
        double rho_C_, rho_D_, rhoOmega_, rho_LV_;
        double LV_max_, LV_min_;
        double emergency_stop_dist_, safe_aera_radius_, collision_avoid_radius_;
        double omega_max_;
        minco::MINCO_S4 mincoOpt_;
        Eigen::MatrixXd initS_;
        double *x_;

        Eigen::Vector3d g_{0.0, 0.0, -9.8};
        Eigen::Vector3d car_p_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d car_v_{Eigen::Vector3d::Zero()}; 
        Trajectory bvp_traj_;
        int iter_times_{0};

        struct violate_cost {
            double cost_p_;
            double cost_v_;
            double cost_a_;
            double cost_j_;
            double cost_d_;
            double cost_l_;
            double cost_c_;
            double cost_omega_;

            inline void reset() {
                cost_p_ = 0.0;
                cost_v_ = 0.0;
                cost_a_ = 0.0;
                cost_j_ = 0.0;
                cost_d_ = 0.0;
                cost_c_ = 0.0;
                cost_omega_ = 0.0;
                cost_l_ = 0.0;
            }

            inline double total_cost() {
                return cost_p_ + cost_v_ + cost_a_ + cost_j_ + cost_d_ + cost_l_ + cost_c_ + cost_omega_;
            }
        };

        violate_cost violate_cost_;

        double land_target_x_ = 0.0;
        double land_target_y_ = 0.0;
        double land_target_z_ = 0.0;
        double uwb_dist_ = 0.0;
        Eigen::Vector3d m_uav_pos_ = Eigen::Vector3d::Zero();

        long long obj_call_ = 0;

        static std::string nowTimeString()
        {
            using namespace std::chrono;
            auto now = system_clock::now();
            std::time_t t = system_clock::to_time_t(now);

            std::tm tm{};
        #ifdef _WIN32
            localtime_s(&tm, &t);
        #else
            localtime_r(&t, &tm);
        #endif

            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");  // Windows 下不要用 :
            return oss.str();
        }

        std::string iter_csv_path_ = "/home/lc/mc_simu_ws/lbfgs_iter_metrics_" + nowTimeString() + ".csv";
        std::string traj_csv_path_ = "/home/lc/mc_simu_ws/traj_metrics_" + nowTimeString() + ".csv";

    public:
        TrajOpt(ros::NodeHandle &nh);
        ~TrajOpt() {}

        Eigen::MatrixXd f_DN(const Eigen::Vector3d &x);

        Eigen::MatrixXd f_D2N(const Eigen::Vector3d &x, const Eigen::Vector3d &y);

        double smoothedL1(const double &x, const double mu, double &grad);

        template <typename EIGENVEC>  
        void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

        template <typename EIGENVEC>
        void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

        template <typename EIGENVEC, typename EIGENVECGD>
        void dRealT_dVirtualT(
            const Eigen::VectorXd &RT,
            const EIGENVEC &VT, 
            const Eigen::VectorXd &gdRT,
            EIGENVECGD &gdVT,
            double wei_time,
            double &costT);

        double gdT2t(double t);

        void bvp(const double &t,
            const Eigen::MatrixXd i_state,
            const Eigen::MatrixXd f_state,
            CoefficientMat &coeffMat);

        bool trans_bvp_traj(Trajectory &traj);

        double getMaxOmega(Trajectory &traj);

        void setLandingParams(const LandingParams &lp);

        bool generate_traj(const Eigen::MatrixXd &iniState,
                            const Eigen::Vector3d &car_p,
                            const Eigen::Vector3d &car_v,
                            const int &N,
                            Trajectory &traj);

        static int progressFunc(void *ptrObj,
                                const double *x,
                                const double *grad,
                                const double fx,
                                const double xnorm,
                                const double gnorm,
                                const double step,
                                int n,
                                int k,
                                int ls);

        static double objectiveFunc(void *ptrObj,
                                    const double *x,
                                    double *grad,
                                    const int n);
                            
        void addTimeIntPenalty(double &cost);

        bool StrongWindAreaGradCostP(const Eigen::Vector3d &p, Eigen::Vector3d &gradp, double &costp);

        bool feasibilityGradCostV(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costv);

        bool feasibilityGradCostA(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa);

        bool feasibilityGradCostJ(const Eigen::Vector3d &j, Eigen::Vector3d &gradj, double &costj);

        bool feasibilityGradCostOmega(const Eigen::Vector3d &a, const Eigen::Vector3d &j, Eigen::Vector3d &grada, Eigen::Vector3d &gradj, double &cost);

        bool CollisionGradCost(const Eigen::Vector3d &p, Eigen::Vector3d &gradp, double &costc);

        bool EmerDistGradCostD(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costd);

        bool LandSmoothGradCost(const Eigen::Vector3d &p, const Eigen::Vector3d &v, Eigen::Vector3d &gradp, Eigen::Vector3d &gradv, double &costl);

        double computeAllowedVmaxGradL(double delta_z, double &dvmax_ddz);

        double computePenaltyWeightGradL(double delta_z, double &dw_ddz);

    };

} // namespace traj_opt