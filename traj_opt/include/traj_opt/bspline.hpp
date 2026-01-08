#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace bspline {

class Bspline_P5 {
public:
  Bspline_P5() {}
  Bspline_P5(const Eigen::MatrixXd &points, const int &order,
             const double &interval) {
    setBspline_P5(points, order, interval);
  }
  ~Bspline_P5() = default;

  // control points: 3 x N (each column is one control point)
  Eigen::MatrixXd control_points_;

  int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd u_; // knots vector
  double interval_;   // knot span \delta t

  double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_;

  inline Eigen::MatrixXd getDerivativeControlPoints() {
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i) {
      double denom = (u_(i + p_ + 1) - u_(i + 1));
      if (fabs(denom) < 1e-12)
        ctp.col(i).setZero();
      else
        ctp.col(i) = p_ * (control_points_.col(i + 1) - control_points_.col(i)) / denom;
    }
    return ctp;
  }

  inline Eigen::MatrixXd get_control_points(void) { return control_points_; }

  // initialize as open-uniform clamped B-spline (passes start/end control points)
  inline void setBspline_P5(const Eigen::MatrixXd &points, const int &order,
                            const double &interval) {
    control_points_ = points;
    p_ = order;          // should be 5 for quintic
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);

    // For quintic B-spline (p=5), we need p+1=6 repeated knots at start and end
    // Clamped B-spline: first p+1 knots = 0, last p+1 knots = (n-p+1)*interval
    // This ensures the curve passes through first and last control points
    double total_time = (n_ - p_ + 1) * interval_;
    
    for (int i = 0; i <= m_; ++i) {
      if (i <= p_) {
        u_(i) = 0.0;
      } else if (i >= m_ - p_) {
        u_(i) = total_time;
      } else {
        u_(i) = (i - p_) * interval_;
      }
    }
  }

  inline void setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }
  inline Eigen::VectorXd getKnot() { return this->u_; }
  inline Eigen::MatrixXd getControlPoint() { return control_points_; }
  inline double getInterval() { return interval_; }

  inline bool getTimeSpan(double &um, double &um_p) {
    if (u_.rows() == 0) return false;
    if (p_ >= u_.rows()) return false;
    if (m_ - p_ >= u_.rows()) return false;

    um = u_(p_);
    um_p = u_(m_ - p_);
    return true;
  }

  inline Eigen::VectorXd evaluateDeBoor(const double &u) {
    double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

    // determine which [ui, ui+1] ub lies in
    int k = p_;
    int k_max = m_ - p_ - 1; // last valid span start
    while (k < k_max) {
      if (u_(k + 1) >= ub) break;
      ++k;
    }

    std::vector<Eigen::VectorXd> d;
    d.reserve(p_ + 1);
    for (int i = 0; i <= p_; ++i) {
      d.push_back(control_points_.col(k - p_ + i));
    }

    for (int r = 1; r <= p_; ++r) {
      for (int i = p_; i >= r; --i) {
        double denom = (u_(i + 1 + k - r) - u_(i + k - p_));
        double alpha = 0.0;
        if (fabs(denom) > 1e-12) {
          alpha = (ub - u_(i + k - p_)) / denom;
        } else {
          alpha = 0.0; // repeated knots
        }
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  inline Eigen::VectorXd evaluateDeBoorT(const double &t) {
    return evaluateDeBoor(t + u_(p_));
  }

  inline Bspline_P5 getDerivative() {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    Bspline_P5 derivative(ctp, p_ - 1, interval_);

    // cut the first and last knot
    if (u_.rows() >= 3) {
      Eigen::VectorXd knot(u_.rows() - 2);
      knot = u_.segment(1, u_.rows() - 2);
      derivative.setKnot(knot);
    }
    return derivative;
  }

  // ============================================================
  // Quintic (p=5) clamped B-spline parameterization
  // For K waypoints, we need K+5 control points for quintic B-spline
  // With clamped conditions: curve passes through first and last waypoints
  // Boundary conditions: start velocity/acc, end velocity/acc
  // ============================================================
  inline void parameterizeToBspline(const double &ts,
                                   const std::vector<Eigen::Vector3d> &point_set,
                                   const std::vector<Eigen::Vector3d> &start_end_derivative,
                                   Eigen::MatrixXd &ctrl_pts) {
    if (ts <= 0) {
      std::cout << "[B-spline]: time step error." << std::endl;
      return;
    }

    if (point_set.size() <= 5) {
      std::cout << "[B-spline]: point set too small for quintic: " << point_set.size()
                << " points." << std::endl;
      return;
    }

    if (start_end_derivative.size() != 4) {
      std::cout << "[B-spline]: derivatives error. Need 4 boundary conditions." << std::endl;
      return;
    }

    int K = (int)point_set.size();  // number of waypoints
    int p = 5;                      // quintic
    int N = K + p;                  // number of control points for clamped quintic
    
    // We have constraints:
    // 1. K waypoint positions
    // 2. 4 boundary conditions (start vel, end vel, start acc, end acc)
    // Total constraints: K + 4
    // But we have N = K+5 unknowns, so we need 1 more constraint
    // Let's add zero jerk at start or end, or use natural spline condition
    
    // For simplicity, we'll use position constraints at waypoints and boundary conditions
    // This gives K+4 equations for K+5 unknowns -> underdetermined
    // We'll add a regularization term or fix one control point
    
    // Alternative approach: use interpolation with K+5 control points
    // and add natural boundary conditions (jerk = 0 at both ends)
    
    int num_constraints = K + 6; // K positions + 2 velocities + 2 accelerations + 2 jerks
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, N);

    // ---- Position constraints at waypoints ----
    // For uniform quintic B-spline, basis function values at integer knots
    // Basis function N_{i,5}(t) evaluated at t = i
    // Values from Cox-deBoor recurrence or known uniform quintic basis
    // These are approximate for uniform B-spline
    double basis_values[6] = {1.0/120, 13.0/60, 11.0/20, 13.0/60, 1.0/120, 0.0};
    
    // Waypoint i (0-based) is evaluated at knot u_{i+p} = u_{i+5}
    // For clamped uniform B-spline, waypoint i corresponds to control points i..i+5
    for (int i = 0; i < K; ++i) {
        for (int j = 0; j < 6; ++j) {
            int cp_idx = i + j;
            if (cp_idx < N) {
                A(i, cp_idx) = basis_values[j];
            }
        }
    }
    
    // Ensure curve passes exactly through first and last waypoints
    // This is important for clamped B-spline
    A(0, 0) = 1.0;
    for (int j = 1; j < 6; ++j) {
        A(0, j) = 0.0;
    }
    
    A(K-1, N-1) = 1.0;
    for (int j = 0; j < 5; ++j) {
        A(K-1, N-6+j) = 0.0;
    }

    // ---- Velocity boundary conditions ----
    // Start velocity: v0 = (P1 - P0) * p / ts
    A(K, 0) = -p / ts;
    A(K, 1) =  p / ts;
    
    // End velocity: vT = (P_{N-1} - P_{N-2}) * p / ts
    A(K+1, N-2) = -p / ts;
    A(K+1, N-1) =  p / ts;

    // ---- Acceleration boundary conditions ----
    double acc_coeff = p * (p-1) / (ts * ts);
    // Start acceleration: a0 = (P2 - 2*P1 + P0) * p*(p-1) / ts^2
    A(K+2, 0) =  acc_coeff;
    A(K+2, 1) = -2 * acc_coeff;
    A(K+2, 2) =  acc_coeff;
    
    // End acceleration: aT = (P_{N-1} - 2*P_{N-2} + P_{N-3}) * p*(p-1) / ts^2
    A(K+3, N-3) =  acc_coeff;
    A(K+3, N-2) = -2 * acc_coeff;
    A(K+3, N-1) =  acc_coeff;

    // ---- Jerk boundary conditions (natural spline) ----
    double jerk_coeff = p * (p-1) * (p-2) / (ts * ts * ts);
    // Start jerk: j0 = (P3 - 3*P2 + 3*P1 - P0) * p*(p-1)*(p-2) / ts^3
    A(K+4, 0) = -jerk_coeff;
    A(K+4, 1) =  3 * jerk_coeff;
    A(K+4, 2) = -3 * jerk_coeff;
    A(K+4, 3) =  jerk_coeff;
    
    // End jerk: jT = (P_{N-1} - 3*P_{N-2} + 3*P_{N-3} - P_{N-4}) * p*(p-1)*(p-2) / ts^3
    A(K+5, N-4) = -jerk_coeff;
    A(K+5, N-3) =  3 * jerk_coeff;
    A(K+5, N-2) = -3 * jerk_coeff;
    A(K+5, N-1) =  jerk_coeff;

    // ---- Right-hand side vectors ----
    Eigen::VectorXd bx(num_constraints), by(num_constraints), bz(num_constraints);
    
    // Position constraints
    for (int i = 0; i < K; ++i) {
        bx(i) = point_set[i].x();
        by(i) = point_set[i].y();
        bz(i) = point_set[i].z();
    }
    
    // Boundary conditions: [v0, vT, a0, aT]
    // start_end_derivative[0] = start velocity
    // start_end_derivative[1] = end velocity  
    // start_end_derivative[2] = start acceleration
    // start_end_derivative[3] = end acceleration
    
    bx(K)   = start_end_derivative[0].x();  // start vel x
    by(K)   = start_end_derivative[0].y();  // start vel y
    bz(K)   = start_end_derivative[0].z();  // start vel z
    
    bx(K+1) = start_end_derivative[1].x();  // end vel x
    by(K+1) = start_end_derivative[1].y();  // end vel y
    bz(K+1) = start_end_derivative[1].z();  // end vel z
    
    bx(K+2) = start_end_derivative[2].x();  // start acc x
    by(K+2) = start_end_derivative[2].y();  // start acc y
    bz(K+2) = start_end_derivative[2].z();  // start acc z
    
    bx(K+3) = start_end_derivative[3].x();  // end acc x
    by(K+3) = start_end_derivative[3].y();  // end acc y
    bz(K+3) = start_end_derivative[3].z();  // end acc z
    
    // Natural spline: zero jerk at boundaries
    bx(K+4) = 0.0;
    by(K+4) = 0.0;
    bz(K+4) = 0.0;
    
    bx(K+5) = 0.0;
    by(K+5) = 0.0;
    bz(K+5) = 0.0;

    // Solve for control points
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // Create control points matrix (3 x N)
    ctrl_pts.resize(3, N);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();
    
    std::cout << "[B-spline]: Quintic B-spline parameterization complete. "
              << K << " waypoints -> " << N << " control points." << std::endl;
  }

  // Rest of the functions remain the same...
  /* check feasibility, adjust time */
  inline void setPhysicalLimits(const double &vel, const double &acc,
                                const double &tolerance) {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  inline bool checkFeasibility(double &ratio, bool show) {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility */
    double max_vel = -1.0;
    double enlarged_vel_lim =
        limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i) {
      double denom = (u_(i + p_ + 1) - u_(i + 1));
      Eigen::VectorXd vel = Eigen::VectorXd::Zero(dimension);
      if (fabs(denom) > 1e-12)
        vel = p_ * (P.col(i + 1) - P.col(i)) / denom;

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim) {

        if (show)
          std::cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose()
                    << std::endl;
        fea = false;

        for (int j = 0; j < dimension; ++j) {
          max_vel = std::max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim =
        limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i) {

      Eigen::VectorXd acc = Eigen::VectorXd::Zero(dimension);

      double d1 = (u_(i + p_ + 2) - u_(i + 2));
      double d2 = (u_(i + p_ + 1) - u_(i + 1));
      double d3 = (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(d1) > 1e-12 && fabs(d2) > 1e-12 && fabs(d3) > 1e-12) {
        acc = p_ * (p_ - 1) *
              ((P.col(i + 2) - P.col(i + 1)) / d1 -
               (P.col(i + 1) - P.col(i)) / d2) / d3;
      }

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim) {

        if (show)
          std::cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose()
                    << std::endl;
        fea = false;

        for (int j = 0; j < dimension; ++j) {
          max_acc = std::max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = std::max(max_vel / limit_vel_,
                     std::sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  inline void lengthenTime(const double &ratio) {
    int num1 = p_;
    int num2 = getKnot().rows() - 1 - p_;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  inline double getTimeSum() {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  inline double getLength(const double &res) {
    double length = 0.0;
    double dur = getTimeSum();
    if (dur <= 0) return 0.0;

    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res) {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  inline double getJerk() {
    Bspline_P5 jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    int max_i = std::min<int>(ctrl_pts.cols(), times.rows() - 1);
    for (int i = 0; i < max_i; ++i) {
      for (int j = 0; j < dimension; ++j) {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }
    return jerk;
  }

  inline void getMeanAndMaxVel(double &mean_v, double &max_v) {
    Bspline_P5 vel = getDerivative();
    double tm, tmp;
    if (!vel.getTimeSpan(tm, tmp)) {
      mean_v = 0.0;
      max_v = 0.0;
      return;
    }

    double max_vel = 0.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel) max_vel = vn;
    }

    mean_v = (num > 0) ? (mean_vel / double(num)) : 0.0;
    max_v = max_vel;
  }

  inline void getMeanAndMaxAcc(double &mean_a, double &max_a) {
    Bspline_P5 acc = getDerivative().getDerivative();
    double tm, tmp;
    if (!acc.getTimeSpan(tm, tmp)) {
      mean_a = 0.0;
      max_a = 0.0;
      return;
    }

    double max_acc = 0.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc) max_acc = an;
    }

    mean_a = (num > 0) ? (mean_acc / double(num)) : 0.0;
    max_a = max_acc;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace bspline