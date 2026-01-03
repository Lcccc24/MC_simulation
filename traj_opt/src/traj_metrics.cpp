// traj_metrics.cpp
#include <traj_opt/traj_metrics.hpp>
#include <traj_opt/poly_traj_utils.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace traj_opt {

TrajMetrics evaluateTrajectory(const Trajectory& traj,
                              int N_piece,
                              bool is_landing,
                              double sample_dt,
                              int first_k)
{
  TrajMetrics m;
  m.N = N_piece;
  m.is_landing = is_landing;
  m.T = traj.getTotalDuration();

  const double T = m.T;
  if (T <= 1e-9) return m;

  const int M = std::max(2, (int)std::ceil(T / sample_dt) + 1);

  // t=0
  Eigen::Vector3d p_prev = traj.getPos(0.0);
  Eigen::Vector3d v_prev = traj.getVel(0.0);
  Eigen::Vector3d a_prev = traj.getAcc(0.0);
  Eigen::Vector3d j_prev = traj.getJer(0.0);
  Eigen::Vector3d s_prev = traj.getSnp(0.0);

  // early drop
  const double t01 = std::min(0.1, T);
  m.z_drop_01s = p_prev.z() - traj.getPos(t01).z();

  m.max_speed = v_prev.norm();
  m.max_acc   = a_prev.norm();
  m.max_jerk  = j_prev.norm();
  m.max_snap  = s_prev.norm();

  for (int k = 1; k < M; ++k) {
    const double t = std::min(T, k * sample_dt);
    const double t_prev = std::min(T, (k - 1) * sample_dt);
    const double dt = t - t_prev;
    if (dt <= 0) continue;

    Eigen::Vector3d p = traj.getPos(t);
    Eigen::Vector3d v = traj.getVel(t);
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d s = traj.getSnp(t);

    m.max_speed = std::max(m.max_speed, v.norm());
    m.max_acc   = std::max(m.max_acc,   a.norm());
    m.max_jerk  = std::max(m.max_jerk,  j.norm());
    m.max_snap  = std::max(m.max_snap,  s.norm());

    // trapezoid integrals
    m.int_acc2   += 0.5 * dt * (a_prev.squaredNorm() + a.squaredNorm());
    m.int_jerk2  += 0.5 * dt * (j_prev.squaredNorm() + j.squaredNorm());
    m.int_snap2  += 0.5 * dt * (s_prev.squaredNorm() + s.squaredNorm());
    m.path_length += 0.5 * dt * (v_prev.norm() + v.norm());

    // first-k displacement (k counts samples, not seconds)
    if (k <= first_k) {
      m.first_k_disp += (p - p_prev).norm();
    }

    p_prev = p; v_prev = v; a_prev = a; j_prev = j; s_prev = s;
  }
  return m;
}

void appendMetricsToCsv(const TrajMetrics& m, const std::string& path)
{
  // 判断文件是否存在/是否空，用于写 header
  bool need_header = false;
  {
    std::ifstream ifs(path);
    need_header = (!ifs.good() || ifs.peek() == std::ifstream::traits_type::eof());
  }

  std::ofstream ofs(path, std::ios::app);
  if (!ofs.is_open()) return;

  if (need_header) {
    ofs << "method,scene_id,success,is_landing,N,T,replan_t,"
           "opt_time_ms,lbfgs_iters,final_objective,"
           "int_acc2,int_jerk2,int_snap2,path_length,"
           "max_speed,max_acc,max_jerk,max_snap,max_omega,"
           "first_k_disp,z_drop_01s,"
           "vio_floor,vio_v,vio_thrust,vio_omega,vio_dist,vio_yaw,vio_perching_collision\n";
  }

  ofs << m.method << "," << m.scene_id << ","
      << (m.success?1:0) << "," << (m.is_landing?1:0) << ","
      << m.N << "," << m.T << "," << m.replan_t << ","
      << m.opt_time_ms << "," << m.lbfgs_iters << "," << m.final_objective << ","
      << m.int_acc2 << "," << m.int_jerk2 << "," << m.int_snap2 << "," << m.path_length << ","
      << m.max_speed << "," << m.max_acc << "," << m.max_jerk << "," << m.max_snap << "," << m.max_omega << ","
      << m.first_k_disp << "," << m.z_drop_01s << ","
      << m.vio_floor << "," << m.vio_v << "," << m.vio_thrust << "," << m.vio_omega << ","  << m.vio_dist << "," << m.vio_yaw << "," << m.vio_perching_collision
      << "\n";
}

}