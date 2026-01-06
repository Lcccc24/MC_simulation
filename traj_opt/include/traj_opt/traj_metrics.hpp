// traj_metrics.hpp
#pragma once
#include <Eigen/Dense>
#include <string>
#include <traj_opt/poly_traj_utils.hpp>


namespace traj_opt {

struct TrajMetrics {
  // identifiers
  std::string method = "minco_lbfgs";
  int scene_id = -1;

  // flags
  bool success = false;
  bool is_landing = false;

  // planning params
  int N = 0;          // piece count
  double T = 0.0;     // total duration
  double replan_t = -1.0;

  // optimization stats
  double opt_time_ms = 0.0;
  int lbfgs_iters = 0;
  double final_objective = 0.0;

  // quality integrals
  double int_acc2 = 0.0;
  double int_jerk2 = 0.0;
  double int_snap2 = 0.0;
  double path_length = 0.0;

  // extrema
  double max_speed = 0.0;
  double max_acc = 0.0;
  double max_jerk = 0.0;
  double max_snap = 0.0;
  double max_omega = 0.0;

  // early-progress metrics (for landing/replan analysis)
  double first_k_disp = 0.0;   // sum displacement of first k samples
  double z_drop_01s = 0.0;     // z(0)-z(0.1s)

  // optional: constraint violation counters
  double vio_p = 0.0;
  double vio_v = 0.0;
  double vio_a = 0.0;
  double vio_j = 0.0;
  double vio_d = 0.0;
  double vio_l = 0.0;
  double vio_omega = 0.0;

};


struct IterMetrics {
  int success = 0;      // 0/1
  int is_landing = 0;   // 0/1

  int iter = 0;         // k
  int ls = 0;           // line search steps
  int n = 0;            // dimension

  double fx = 0.0;
  double xnorm = 0.0;
  double gnorm = 0.0;
  double step = 0.0;

  // 分项（来自 objectiveFunc 的当前一次 evaluation 结果）
  double vio_p = 0.0;
  double vio_v = 0.0;
  double vio_a = 0.0;
  double vio_j = 0.0;
  double vio_d = 0.0;
  double vio_l = 0.0;
  double vio_omega = 0.0;

  // 可选：objectiveFunc调用计数
  long long obj_calls = 0;
};


TrajMetrics evaluateTrajectory(const Trajectory& traj,
                              int N_piece,
                              bool is_landing,
                              double sample_dt,
                              int first_k);

void appendMetricsToCsv(const TrajMetrics& m, const std::string& path);
void appendIterMetricsToCsv(const IterMetrics& m, const std::string& path);

}