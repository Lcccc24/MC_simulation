#include <traj_opt/traj_opt.h>
#include <traj_opt/traj_metrics.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt
{

  TrajOpt::TrajOpt(ros::NodeHandle &nh)
  {
    nh.param("traj_opt/is_landing", is_landing_, false);
    // nh.getParam("N", N_);
    nh.param("traj_opt/K", K_, 16);
    // load dynamic paramters
    nh.param("traj_opt/vmax", vmax_, 3.0);
    nh.param("traj_opt/amax", amax_, 3.0);
    nh.param("traj_opt/jmax", jmax_, 3.0);
    nh.param("traj_opt/omega_max", omega_max_, 3.0);
    nh.param("traj_opt/rhoT", rhoT_, 100000.0);
    nh.param("traj_opt/rhoP", rhoP_, 10000000.0);
    nh.param("traj_opt/rhoV", rhoV_, 1000.0);
    nh.param("traj_opt/rhoA", rhoA_, 1000.0);
    nh.param("traj_opt/rhoJ", rhoJ_, 1000.0);
    nh.param("traj_opt/rhoD", rho_D_, 100000.0);
    nh.param("traj_opt/rhoC", rho_C_, 100000.0);
    nh.param("traj_opt/rhoLV", rho_LV_, 100000.0);
    nh.param("traj_opt/rhoOmega", rhoOmega_, 100000.0);
    nh.param("traj_opt/LV_max", LV_max_, 0.5);
    nh.param("traj_opt/LV_min", LV_min_, 0.1);
    nh.param("traj_opt/emergency_stop_dist", emergency_stop_dist_, 1.0);
    nh.param("traj_opt/safe_aera_radius", safe_aera_radius_, 0.2);
    nh.param("traj_opt/collision_avoid_radius", collision_avoid_radius_, 0.8);
    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
  }

  void TrajOpt::setLandingParams(const LandingParams &lp) {
    is_landing_ = lp.is_landing;
    uwb_dist_ = lp.uwb_dist;
    m_uav_pos_.x() = lp.m_uav_pos.x();
    m_uav_pos_.y() = lp.m_uav_pos.y();
    m_uav_pos_.z() = lp.m_uav_pos.z();

    if (is_landing_) {
      land_target_x_ = lp.land_x;
      land_target_y_ = lp.land_y;
      land_target_z_ = lp.land_z;
    }
  }

  //推力方向导数
  Eigen::MatrixXd TrajOpt::f_DN(const Eigen::Vector3d &x)
  {
    const double eps = 1e-9;

    if (!x.allFinite()) {
      std::cerr << "f_DN: x is not finite: " << x.transpose() << std::endl;
      return Eigen::Matrix3d::Zero();
    }

    double x_norm_2 = x.squaredNorm();

    if (!(x_norm_2 > eps)) { 
      return Eigen::Matrix3d::Zero();
    }

    return (Eigen::Matrix3d::Identity() - x * x.transpose() / x_norm_2) / std::sqrt(x_norm_2);
  }

  //推力方向导数
  Eigen::MatrixXd TrajOpt::f_D2N(const Eigen::Vector3d &x, const Eigen::Vector3d &y)
  {
    double x_norm_2 = x.squaredNorm();
    double x_norm_3 = x_norm_2 * x.norm();
    Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
    return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
  }
  
  // SECTION  variables transformation and gradient transmission
  //max(x,0)的C2光滑化函数 
  double TrajOpt::smoothedL1(const double &x, const double mu, double &grad)
  {
    if (x < 0.0)
    {
      return 0.0;
    }
    else if (x > mu)
    {
      grad = 1.0;
      return x - 0.5 * mu;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return mumxd2 * sqrxdmu * xdmu;
    }
  }

  template <typename EIGENVEC>  
  void TrajOpt::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT) {
    for (int i = 0; i < RT.size(); ++i) {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void TrajOpt::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT) {
    for (int i = 0; i < VT.size(); ++i) {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) 
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  double TrajOpt::gdT2t(double t)
  {
    if (t > 0)
    {
      return t + 1.0;
    }
    else
    {
      double denSqrt = (0.5 * t - 1.0) * t + 1.0;
      return (1.0 - t) / (denSqrt * denSqrt);
    }
  }


  template <typename EIGENVEC, typename EIGENVECGD>
  void TrajOpt::dRealT_dVirtualT(
    const Eigen::VectorXd &RT,
    const EIGENVEC &VT, 
    const Eigen::VectorXd &gdRT,
    EIGENVECGD &gdVT,
    double wei_time,
    double &costT)
  {
    const int N = VT.size();
    gdVT.resize(N);

    for (int i = 0; i < N; ++i)
    {
      const double dRt_dVt = gdT2t(VT(i));
      gdVT(i) = (gdRT(i) + wei_time) * dRt_dVt;
    }

    costT = RT.sum() * wei_time;
  }

  void TrajOpt::bvp(const double &t,
                  const Eigen::MatrixXd i_state,
                  const Eigen::MatrixXd f_state,
                  CoefficientMat &coeffMat)
  {
    double t1 = t;
    double t2 = t1 * t1;
    double t3 = t2 * t1;
    double t4 = t2 * t2;
    double t5 = t3 * t2;
    double t6 = t3 * t3;
    double t7 = t4 * t3;
    CoefficientMat boundCond;
    boundCond.leftCols(4) = i_state;
    boundCond.rightCols(4) = f_state;

    coeffMat.col(0) = (boundCond.col(7) / 6.0 + boundCond.col(3) / 6.0) * t3 +
                      (-2.0 * boundCond.col(6) + 2.0 * boundCond.col(2)) * t2 +
                      (10.0 * boundCond.col(5) + 10.0 * boundCond.col(1)) * t1 +
                      (-20.0 * boundCond.col(4) + 20.0 * boundCond.col(0));
    coeffMat.col(1) = (-0.5 * boundCond.col(7) - boundCond.col(3) / 1.5) * t3 +
                      (6.5 * boundCond.col(6) - 7.5 * boundCond.col(2)) * t2 + 
                      (-34.0 * boundCond.col(5) - 36.0 * boundCond.col(1)) * t1 +
                      (70.0 * boundCond.col(4) - 70.0 * boundCond.col(0));
    coeffMat.col(2) = (0.5 * boundCond.col(7) + boundCond.col(3)) * t3 +
                      (-7.0 * boundCond.col(6) + 10.0 * boundCond.col(2)) * t2 +
                      (39.0 * boundCond.col(5) + 45.0 * boundCond.col(1)) * t1 +
                      (-84.0 * boundCond.col(4) + 84.0 * boundCond.col(0));
    coeffMat.col(3) = (-boundCond.col(7) / 6.0 - boundCond.col(3) / 1.5) * t3 +
                      (2.5 * boundCond.col(6) - 5.0 * boundCond.col(2)) * t2 +
                      (-15.0 * boundCond.col(5) - 20.0 * boundCond.col(1)) * t1 +
                      (35.0 * boundCond.col(4) - 35.0 * boundCond.col(0));
    coeffMat.col(4) = boundCond.col(3) / 6.0;
    coeffMat.col(5) = boundCond.col(2) / 2.0;
    coeffMat.col(6) = boundCond.col(1);
    coeffMat.col(7) = boundCond.col(0);

    coeffMat.col(0) = coeffMat.col(0) / t7;
    coeffMat.col(1) = coeffMat.col(1) / t6;
    coeffMat.col(2) = coeffMat.col(2) / t5;
    coeffMat.col(3) = coeffMat.col(3) / t4;
  }

  double TrajOpt::getMaxOmega(Trajectory &traj)
  {
    const double dt = 0.05;
    double max_omega = 0;
    const double T = traj.getTotalDuration();

    for (double t = 0; t < T; t += dt)
    {
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);

      if (!a.allFinite() || !j.allFinite() || !g_.allFinite()) {
        std::cerr << "[NaN] t=" << t
                  << " a=" << a.transpose()
                  << " j=" << j.transpose()
                  << " g=" << g_.transpose()
                  << " T=" << T
                  << std::endl;
        break;
      }

      Eigen::Vector3d thrust = a - g_;
      if (!thrust.allFinite()) {
        std::cerr << "[NaN] t=" << t << " thrust=" << thrust.transpose() << std::endl;
        continue;
      }

      Eigen::Vector3d zb_dot = f_DN(thrust) * j;
      if (!zb_dot.allFinite()) continue;

      double omega12 = zb_dot.norm();
      if (std::isfinite(omega12) && omega12 > max_omega) max_omega = omega12;
    }
    return max_omega;
  }


  bool TrajOpt::trans_bvp_traj(Trajectory &traj)
  {
    traj = bvp_traj_;
    return true;
  }

  double TrajOpt::objectiveFunc(void *ptrObj,
                                  const double *x,
                                  double *grad,
                                  const int n)
  {
    
    auto* obj = static_cast<TrajOpt*>(ptrObj);
    obj->iter_times_++;
    obj->obj_call_++;
    obj->violate_cost_.reset();
    Eigen::Map<const Eigen::VectorXd> Virtual_T(x, obj->dim_t_);
    Eigen::Map<Eigen::VectorXd> grad_vt(grad, obj->dim_t_);
    Eigen::Map<const Eigen::MatrixXd> P(x + obj->dim_t_, 3, obj->dim_p_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad + obj->dim_t_, 3, obj->dim_p_);

    Eigen::VectorXd Dur_T(obj->N_);  
    obj->VirtualT2RealT(Virtual_T, Dur_T);
    for (int i = 0; i < obj->N_; ++i) {
      if (Dur_T(i) < 1e-3) Dur_T(i) = 1e-3;
    }
    const double total_t = Dur_T.sum();

    Eigen::MatrixXd tailS(3, 4);
    tailS.col(0) = obj->car_p_ + obj->car_v_ * total_t;
    tailS.col(1) = obj->car_v_;
    tailS.col(2).setZero();
    tailS.col(3).setZero();

    obj->mincoOpt_.generate(obj->initS_, tailS, P, Dur_T);
    double cost = obj->mincoOpt_.getTrajSnapCost();
    obj->mincoOpt_.calGrads_CT();
    obj->addTimeIntPenalty(cost);
    obj->mincoOpt_.calGrads_PT();
    gradP = obj->mincoOpt_.gdP;
    double costT = 0;
    obj->dRealT_dVirtualT(Dur_T, Virtual_T, obj->mincoOpt_.gdT, grad_vt, obj->rhoT_, costT);
    cost += costT;

    return cost;
  }

  int TrajOpt::progressFunc(void *ptrObj,
                             const double *x,
                             const double *grad,
                             const double fx,
                             const double xnorm,
                             const double gnorm,
                             const double step,
                             int n,
                             int k,
                             int ls)
  {
    auto* obj = static_cast<TrajOpt*>(ptrObj);

    IterMetrics row;
    row.success = 0; // 迭代过程先写 0，最终成功你会在 traj_metrics.csv 里记录
    row.is_landing = obj->is_landing_ ? 1 : 0; 

    row.iter = k;
    row.ls   = ls;
    row.n    = n;

    row.fx    = static_cast<double>(fx);
    row.xnorm = static_cast<double>(xnorm);
    row.gnorm = static_cast<double>(gnorm);
    row.step  = static_cast<double>(step);

    // 关键：这些值必须在 objectiveFunc 里“本次评估”计算后写入 obj->violate_cost_
    row.vio_p     = obj->violate_cost_.cost_p_;
    row.vio_v     = obj->violate_cost_.cost_v_;
    row.vio_a     = obj->violate_cost_.cost_a_;
    row.vio_j     = obj->violate_cost_.cost_j_;
    row.vio_d     = obj->violate_cost_.cost_d_;
    row.vio_l     = obj->violate_cost_.cost_l_;
    row.vio_c     = obj->violate_cost_.cost_c_;
    row.vio_omega = obj->violate_cost_.cost_omega_;
    row.obj_calls = obj->obj_call_;

    appendIterMetricsToCsv(row, obj->iter_csv_path_);
    return 0;
  }

  /*
  iniState: 初始状态 包含位置速度等信息
  car_p: 目标位置
  car_v: 目标速度
  N: 轨迹分段数
  t_replan: 重规划时间 t_replan默认为-1时，不进行重规划
  traj: 轨迹对象，用于存储生成的轨迹信息
  */
  bool TrajOpt::generate_minco_traj(const Eigen::MatrixXd &iniState,
                              const Eigen::Vector3d &car_p,
                              const Eigen::Vector3d &car_v,
                              const int &N,
                              Trajectory &traj)
  {
    N_ = N;  
    //时间维度
    dim_t_ = N_;
    //位置控制点的数量（轨迹用 N-1 个控制点表示）
    dim_p_ = N_ - 1;
    //时间+路点
    x_ = new double[dim_t_ + 3 * dim_p_]; 
    //时间赋值 转化为无约束后送入优化
    Eigen::Map<Eigen::VectorXd> Virtual_T(x_, dim_t_);
    //路点赋值 
    Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_);
    //真实时间 duration
    Eigen::VectorXd Dur_T(N_);

    car_p_ = car_p;
    car_v_ = car_v;

    // NOTE set boundary conditions
    initS_ = iniState;

    mincoOpt_.reset(N_);

    // 利用 BVP 解在均匀时间点采样，得到初始控制点 P，为后续优化提供合理初始值
    Eigen::MatrixXd bvp_i = initS_;
    Eigen::MatrixXd bvp_f(3, 4);
    bvp_f.col(0) = car_p_;
    bvp_f.col(1) = car_v_;
    bvp_f.col(2).setZero();
    bvp_f.col(3).setZero();
    //先按最大速度求T_bvp,后面在while循环减小到 到达要求为止
    double t_bvp = (bvp_f.col(0) - bvp_i.col(0)).norm() / vmax_;
    CoefficientMat coeffMat;
    double max_omega = 0;
    do {
      t_bvp += 1.0;
      //假设目标以恒定速度 car_v_ 移动，新的终止位置是初始位置加上时间内的位移。
      bvp_f.col(0) = car_p_ + car_v_ * t_bvp;
      //bvp求出起点到终点这段轨迹的多项式系数矩阵，并将其存储在 coeffMat
      bvp(t_bvp, bvp_i, bvp_f, coeffMat);
      std::vector<double> durs{t_bvp};
      std::vector<CoefficientMat> coeffs{coeffMat};
      Trajectory traj(durs, coeffs);
      max_omega = getMaxOmega(traj);

      bvp_traj_ = traj;
    } while (max_omega > 1.5 * omega_max_);
    //创建一个 8 维向量 tt，并将最后一个元素设为 1.0

    Eigen::VectorXd tt(8);
    //tt(7)为t零次方=1 倒序填充
    tt(7) = 1.0;
    for (int i = 1; i < N_; ++i) {
      //计算第 i 个控制点对应的归一化时间
      double tt0 = (i * 1.0 / N_) * t_bvp;
      for (int j = 6; j >= 0; j -= 1) {
        tt(j) = tt(j + 1) * tt0;
      }
      //计算当前控制点对应的轨迹位置，并将其存储在 P 的第 i 列中
      //把bvp算出来的轨迹分成N个控制点，每个控制点对应一个时间tt
      P.col(i - 1) = coeffMat * tt;
    }
    //t_bvp / N_：平均每段轨迹的时间
    Dur_T.setConstant(t_bvp / N_);
    //std::cout  << "Init Dur_T: " << Dur_T.transpose() << std::endl;
    RealT2VirtualT(Dur_T, Virtual_T);

    // NOTE optimization
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 32;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.min_step = 1e-16;
    lbfgs_params.delta = 1e-4;
    lbfgs_params.line_search_type = 0;
    double minObjective;
    int opt_ret = 0;

    //用于记录优化过程时间
    auto tic = std::chrono::steady_clock::now();

    //第一个参数 优化变量个数
    //minObjective 存储最小函数值的变量
    //objectiveFunc 计算目标函数和梯度的函数
    //earlyExit 满足某些条件时退出优化的函数
    //lbfgs_params 自定义参数
    opt_ret = lbfgs::lbfgs_optimize(
                        dim_t_ + 3 * dim_p_,
                        x_, 
                        &minObjective,
                        TrajOpt::objectiveFunc, 
                        nullptr,
                        TrajOpt::progressFunc, 
                        this, 
                        &lbfgs_params);

    //用于记录优化过程时间
    auto toc = std::chrono::steady_clock::now();

    if(opt_ret>=0) {
      // std::cout << "\033[32m>ret: " << opt_ret << "\033[0m" << std::endl;
    } else {
      // 优化失败 清理内存并返回false
      auto err_msg = lbfgs::lbfgs_strerror(opt_ret);
      std::cout << "traj opt err: " << err_msg << "optimization failed" << std::endl;
      delete[] x_;  
      return false;
    }

    // 计算优化后的时间步长和总时间
    //此时参数都经过优化器处理 tailS跟objectiveFunc函数最后一次输出的tails相同
    VirtualT2RealT(Virtual_T, Dur_T);
    for (int i = 0; i < N_; ++i) {
      if (Dur_T(i) < 1e-3) Dur_T(i) = 1e-3;
    }
    double total_t = Dur_T.sum();
    Eigen::MatrixXd tailS(3, 4);
    // 静止时 car_v_为0
    tailS.col(0) = car_p_ + car_v_ * total_t;
    tailS.col(1) = car_v_;
    tailS.col(2).setZero();
    tailS.col(3).setZero();
    mincoOpt_.generate(initS_, tailS, P, Dur_T);
    traj = mincoOpt_.getTraj();
    //std::cout  << "Optimal Dur_T: " << Dur_T.transpose() << std::endl;

    TrajMetrics met = evaluateTrajectory(traj, N_, is_landing_, 0.01 /*100Hz*/, 5);
    met.success = (opt_ret >= 0);
    met.opt_time_ms = std::chrono::duration<double, std::milli>(toc - tic).count();
    met.lbfgs_iters = iter_times_;
    met.final_objective = minObjective;
    met.max_omega = getMaxOmega(traj);
    met.method = "minco_lbfgs_piece" + std::to_string(N_);
    met.vio_p = violate_cost_.cost_p_;
    met.vio_v = violate_cost_.cost_v_;
    met.vio_a = violate_cost_.cost_a_;
    met.vio_j = violate_cost_.cost_j_;
    met.vio_d = violate_cost_.cost_d_;
    met.vio_l = violate_cost_.cost_l_;
    met.vio_c = violate_cost_.cost_c_;
    met.vio_omega = violate_cost_.cost_omega_;
    appendMetricsToCsv(met, traj_csv_path_);

    delete[] x_;
    return true;
  }

  void TrajOpt::addTimeIntPenalty(double &cost)
  {
    Eigen::Vector3d pos, vel, acc, jer, snp;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::Matrix<double, 8, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
    Eigen::VectorXd costs(8);
    costs.setZero();
    double omg;

    for (int i = 0; i < N_; ++i)
    {
      const auto &c = mincoOpt_.b.block<8, 3>(i * 8, 0);
      step = mincoOpt_.T1(i) / K_;
      s1 = 0.0;

      // <= ???
      for (int j = 0; j <= K_; ++j) {

        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s4 * s2;
        s7 = s4 * s3;
        //pvaj snap 对应的beta
        beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
        alpha = 1.0 / K_ * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snp = c.transpose() * beta4;
 
        omg = (j == 0 || j == K_) ? 0.5 : 1.0;

        if (!is_landing_ && feasibilityGradCostV(vel, gradv, costs(0))) {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaVc;
          mincoOpt_.gdT(i) += omg * (costs(0) / K_ + step * gradViolaVt);
          violate_cost_.cost_v_ += omg * step * costs(0);
        }

        if (feasibilityGradCostA(acc, grada, costs(1))) {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaAc;
          mincoOpt_.gdT(i) += omg * (costs(1) / K_ + step * gradViolaAt);
          violate_cost_.cost_a_ += omg * step * costs(1);
        }

        if (feasibilityGradCostJ(jer, gradj, costs(2))) {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snp;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaJc;
          mincoOpt_.gdT(i) += omg * (costs(2) / K_ + step * gradViolaJt);
          violate_cost_.cost_j_ += omg * step * costs(2);
        }

        if (feasibilityGradCostOmega(acc, jer, grada, gradj, costs(3))) {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          gradViolaJt = alpha * gradj.transpose() * snp;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * (gradViolaAc + gradViolaJc);
          mincoOpt_.gdT(i) += omg * (costs(3) / K_ + step * (gradViolaAt + gradViolaJt));
          violate_cost_.cost_omega_ += omg * step * costs(3);
        }

        if (!is_landing_ && EmerDistGradCostD(vel, gradv, costs(4))) {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaVc;
          mincoOpt_.gdT(i) += omg * (costs(4) / K_ + step * gradViolaVt);
          violate_cost_.cost_d_ += omg * step * costs(4);
        }

        if (!is_landing_ && CollisionGradCost(pos, gradp, costs(5))) {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaPc;
          mincoOpt_.gdT(i) += omg * (costs(5) / K_ + step * gradViolaPt);
          violate_cost_.cost_c_ += omg * step * costs(5);
        }

        if (is_landing_ && StrongWindAreaGradCostP(pos, gradp, costs(6))) {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaPc;
          mincoOpt_.gdT(i) += omg * (costs(6) / K_ + step * gradViolaPt);
          violate_cost_.cost_p_ += omg * step * costs(6);
        }

        if (is_landing_ && LandSmoothGradCost(pos, vel, gradp, gradv, costs(7))) {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          gradViolaVt = alpha * gradv.transpose() * acc;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * (gradViolaPc + gradViolaVc);
          mincoOpt_.gdT(i) += omg * (costs(7) / K_ + step * (gradViolaPt + gradViolaVt));
          violate_cost_.cost_l_ += omg * step * costs(7);
        }

        s1 += step;
      }
    }
    cost += violate_cost_.total_cost();
  }

  bool TrajOpt::StrongWindAreaGradCostP(const Eigen::Vector3d &p, Eigen::Vector3d &gradp, double &costp) {
    constexpr double mu = 0.05;
    double ppenx = std::fabs(p.x() - land_target_x_) - safe_aera_radius_;
    double ppeny = std::fabs(p.y() - land_target_y_) - safe_aera_radius_;

    gradp.setZero();
    costp = 0.0;

    if (ppenx < 0 && ppeny < 0) {
      return false;
    }

    if (ppenx > 0) {
      double dx = 0.0;
      costp += rhoP_ * smoothedL1(ppenx, mu, dx);
      gradp.x() = rhoP_ * dx * ((p.x() >= land_target_x_) ? 1.0 : -1.0);
    }

    if (ppeny > 0) {
      double dy = 0.0;
      costp += rhoP_ * smoothedL1(ppeny, mu, dy);
      gradp.y() = rhoP_ * dy * ((p.y() >= land_target_y_) ? 1.0 : -1.0);
    }

    return true;
  }

  bool TrajOpt::feasibilityGradCostV(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costv) {
    constexpr double mu = 0.1;
    double vpen = v.squaredNorm() - vmax_ * vmax_;

    if (vpen  < 0) {
      gradv.setZero();
      costv = 0.0;
      return false;
    }

    double d = 0.0;
    costv = rhoV_ * smoothedL1(vpen, mu, d);
    gradv = rhoV_ * d * 2 * v;
    return true;
    // gradv = rhoV_ * 6 * vpen * vpen * v;
    // costv = rhoV_ * vpen * vpen * vpen;
    // return true;
  }

  bool TrajOpt::feasibilityGradCostA(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa) {
    constexpr double mu = 0.05;
    double apen = a.squaredNorm() - amax_ * amax_;

    if (apen  < 0) {
      grada.setZero();
      costa = 0.0;
      return false;
    }

    double d = 0.0;
    costa = rhoA_ * smoothedL1(apen, mu, d);
    grada = rhoA_ * d * 2 * a;
    return true;
    // grada = rhoA_ * 6 * apen * apen * a;
    // costa = rhoA_ * apen * apen * apen;
    // return true;
  }

  bool TrajOpt::feasibilityGradCostJ(const Eigen::Vector3d &j, Eigen::Vector3d &gradj, double &costj) {
    constexpr double mu = 0.05;
    double jpen = j.squaredNorm() - jmax_ * jmax_;

    if (jpen  < 0) {
      gradj.setZero();
      costj = 0.0;
      return false;
    }

    double d = 0.0;
    costj = rhoJ_ * smoothedL1(jpen, mu, d);
    gradj = rhoJ_ * d * 2 * j;
    return true;
    // gradj = rhoJ_ * 6 * jpen * jpen * j;
    // costj = rhoJ_ * jpen * jpen * jpen;
    // return true;
  }


  // using hopf fibration:
  // [a,b,c] = thrust.normalized()
  // \omega_1 = sin(\phi) \dot{a] - cos(\phi) \dot{b} - (a sin(\phi) - b cos(\phi)) (\dot{c}/(1+c))
  // \omega_2 = cos(\phi) \dot{a] - sin(\phi) \dot{b} - (a cos(\phi) - b sin(\phi)) (\dot{c}/(1+c))
  // \omega_3 = (b \dot{a} - a \dot(b)) / (1+c)
  // || \omega_12 ||^2 = \omega_1^2 + \omega_2^2 = \dot{a}^2 + \dot{b}^2 + \dot{c}^2
  //hopf 纤维化
  bool TrajOpt::feasibilityGradCostOmega(const Eigen::Vector3d &a, const Eigen::Vector3d &j, Eigen::Vector3d &grada, Eigen::Vector3d &gradj, double &cost) {
    constexpr double mu = 0.05;
    Eigen::Vector3d thrust_f = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust_f) * j;
    double omega_12_sq = zb_dot.squaredNorm();
    double open = omega_12_sq - omega_max_ * omega_max_;

    if (open < 0) {
      grada.setZero();
      gradj.setZero();
      cost = 0.0;
      return false;
    }

    double d = 0.0;
    cost = rhoOmega_ * smoothedL1(open, mu, d);
    gradj = rhoOmega_ * d * f_DN(thrust_f).transpose() * 2 * zb_dot;
    grada = rhoOmega_ * d * f_D2N(thrust_f, j).transpose() * 2 * zb_dot;
    return true;
  }

  bool TrajOpt::LandSmoothGradCost(const Eigen::Vector3d &p, const Eigen::Vector3d &v, Eigen::Vector3d &gradp, Eigen::Vector3d &gradv, double &costl) {
    constexpr double mu = 0.1;
    double delta_z = p.z() - land_target_z_;
    delta_z = std::max(delta_z, 0.0);
    
    double dv, dw_ddz, dvmax_ddz = 0.0;
    double allowed_vmax = computeAllowedVmaxGradL(delta_z, dvmax_ddz);
    double lpen = v.squaredNorm() - allowed_vmax * allowed_vmax;

    if (lpen < 0) {
      gradp.setZero();
      gradv.setZero();
      costl = 0.0;
      return false;
    }

    double penalty_weight = computePenaltyWeightGradL(delta_z, dw_ddz);
    // phi = smoothedL1(lpen), dphi/dlpen = dv
    double phi = smoothedL1(lpen, mu, dv);
    costl = penalty_weight * phi;
    // ∂cost/∂v = weight * dphi/dlpen * ∂lpen/∂v 
    gradv = penalty_weight * dv * 2 * v ;
    // ∂cost/∂z = w'(dz)*phi + w(dz)*dphi/dlpen * ∂lpen/∂z
    // lpen = v^2 - vmax(dz)^2  =>  ∂lpen/∂z = -2*vmax*dvmax/dz
    gradp.setZero();
    gradp.z() = dw_ddz * phi + penalty_weight * dv * -2 * allowed_vmax * dvmax_ddz;
    return true;
  }

  double TrajOpt::computeAllowedVmaxGradL(double delta_z, double &dvmax_ddz) {
    // ratio = dz/(dz+1), dr/dz = 1/(dz+1)^2
    const double denom = delta_z + 1.0;
    const double ratio = delta_z / denom;
    const double A = (LV_max_ - LV_min_);
    const double k = 3.0; 
    const double e = std::exp(-k * ratio);
    double vmax = LV_min_ + A * (1.0 - e);

    const double dratio_ddz = 1.0 / (denom * denom);
    // d/dz [1 - exp(-k r)] = exp(-k r) * k * dr/dz
    dvmax_ddz = A * (e * k * dratio_ddz);

    return vmax;
  }
  
  double TrajOpt::computePenaltyWeightGradL(double delta_z, double &dw_ddz) {
    const double base = rho_LV_;
    const double eps = 0.05;
    const double k = 5.0; 
    const double denom = delta_z + eps;
    double weight = base * (1.0 + k / denom);

    // dw/dz = base * (-k) / (dz+eps)^2
    dw_ddz = base * (-k) / (denom * denom);

    return weight;
  }

  bool TrajOpt::EmerDistGradCostD(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costd) {
    constexpr double mu = 0.01;
    double dpen = emergency_stop_dist_ - uwb_dist_;

    if (dpen < 0) {
      gradv.setZero();
      costd = 0.0;
      return false;
    }

    double dd = 0.0;
    double phi = smoothedL1(dpen, mu, dd);
    costd = rho_D_ * v.squaredNorm() * phi;
    gradv = rho_D_ * phi * 2 * v;
    return true;
  }

  bool TrajOpt::CollisionGradCost(const Eigen::Vector3d &p, Eigen::Vector3d &gradp, double &costc) {
    constexpr double mu = 0.01;
    const Eigen::Vector3d p_err = p - m_uav_pos_;
    const double ppen = collision_avoid_radius_ * collision_avoid_radius_ - p_err.squaredNorm();

    if (ppen < 0) {
      gradp.setZero();
      costc = 0.0;
      return false;
    }

    double dp = 0.0;
    costc = rho_C_ * smoothedL1(ppen, mu, dp);
    gradp = rho_C_ * dp * -2 * p_err;
    return true;
  }

  // bool TrajOpt::generate_bspline_traj(const Eigen::MatrixXd &iniState,
  //                                     const Eigen::Vector3d &car_p,
  //                                     const Eigen::Vector3d &car_v,
  //                                     const int &N,
  //                                     Bspline_P5 &spline_out)
  // {
  //   // -------- 0) cache inputs ----------
  //   car_p_ = car_p;
  //   car_v_ = car_v;
  //   initS_ = iniState;
  //   N_ = N;

  //   // -------- 1) build an initial BVP trajectory & decide total time ----------
  //   Eigen::MatrixXd bvp_i = initS_;
  //   Eigen::MatrixXd bvp_f(3, 4);
  //   bvp_f.col(0) = car_p_;
  //   bvp_f.col(1) = car_v_;
  //   bvp_f.col(2).setZero();
  //   bvp_f.col(3).setZero();

  //   double t_bvp = (bvp_f.col(0) - bvp_i.col(0)).norm() / vmax_;
  //   CoefficientMat coeffMat;
  //   double max_omega = 0.0;

  //   do {
  //     t_bvp += 1.0;
  //     bvp_f.col(0) = car_p_ + car_v_ * t_bvp;
  //     bvp(t_bvp, bvp_i, bvp_f, coeffMat);

  //     std::vector<double> durs{t_bvp};
  //     std::vector<CoefficientMat> coeffs{coeffMat};
  //     Trajectory tmp_traj(durs, coeffs);

  //     // 用 poly sampler 评估 omega（需要你有 PolyTrajSampler）
  //     // PolyTrajSampler polyS(tmp_traj);
  //     // max_omega = getMaxOmega(polyS);

  //     // 如果你暂时还没改 sampler，这里先用你旧的 getMaxOmega(tmp_traj)
  //     max_omega = getMaxOmega(tmp_traj);   // TODO: 等你统一 sampler 后换掉

  //     bvp_traj_ = tmp_traj;
  //   } while (max_omega > 1.5 * omega_max_);

  //   const double total_t = t_bvp;

  //   // -------- 2) choose spline params ----------
  //   const int order = 5;               // 五次（jerk连续）
  //   const int M = std::max(N_ + order, order + 6); // 控制点数（列数），可自行调整
  //   const int K = M - 2;               // parameterizeToBspline 需要 point_set.size() = K (>=4)
  //   if (K < 4) {
  //     std::cout << "[Bspline] K too small." << std::endl;
  //     return false;
  //   }

  //   const double ts = total_t / (K - 1);     // 采样间隔，也作为 knot interval 初值

  //   // -------- 3) sample points from initial (BVP) trajectory ----------
  //   std::vector<Eigen::Vector3d> point_set;
  //   point_set.reserve(K);

  //   for (int i = 0; i < K; ++i) {
  //     double t = i * ts;

  //     // TODO: 用你 Trajectory 的取位置函数替换
  //     // Eigen::Vector3d p = bvp_traj_.getPos(t);

  //     Eigen::Vector3d p = bvp_traj_.getPos(t);  // 你若没有这个接口就改成你实际接口
  //     point_set.push_back(p);
  //   }

  //   // 强制最后一点为“移动目标在 total_t 时刻的位置”
  //   point_set.back() = car_p_ + car_v_ * total_t;

  //   // -------- 4) boundary derivatives for parameterizeToBspline ----------
  //   // start_end_derivative size must be 4 (your code checks that).
  //   std::vector<Eigen::Vector3d> der(4);
  //   der[0] = initS_.col(1);                 // start vel
  //   der[1] = car_v_;                        // end vel
  //   der[2] = initS_.col(2);                 // start acc (or zero if unavailable)
  //   der[3] = Eigen::Vector3d::Zero();       // end acc (can be tuned)

  //   // -------- 5) parameterize to get initial control points ----------
  //   Eigen::MatrixXd ctrl_pts; // 3 x (K+2) == 3 x M
  //   {
  //     bspline::Bspline_P5 tmp;
  //     tmp.parameterizeToBspline(ts, point_set, der, ctrl_pts);
  //   }

  //   if (ctrl_pts.rows() != 3 || ctrl_pts.cols() != K + 2) {
  //     std::cout << "[Bspline] ctrl_pts size mismatch." << std::endl;
  //     return false;
  //   }

  //   // -------- 6) create spline & enforce feasibility by time scaling ----------
  //   Bspline_P5 Bspline_P5(ctrl_pts, order, ts);
  //   Bspline_P5.setPhysicalLimits(vmax_, amax_, 0.05);

  //   double ratio = 1.0;
  //   int fea_iter = 0;
  //   while (!spline.checkFeasibility(ratio, false) && fea_iter++ < 10) {
  //     spline.lengthenTime(std::max(ratio, 1.05));
  //   }

  //   // -------- 7) (Optional) optimize control points via L-BFGS ----------
  //   // 为了先跑通对比，这里先不做 L-BFGS 控制点优化；你想要的话我下面给你完整骨架。
  //   // spline_out = optimized spline
  //   spline_out = spline;

  //   // -------- 8) unified evaluation (建议你尽快改成 sampler 版本) ----------
  //   BsplineTrajSampler bsS(spline_out);

  //   auto tic = std::chrono::steady_clock::now();
  //   TrajMetrics met = evaluateTrajectory(bsS, N_, is_landing_, 0.01 /*100Hz*/, 5);
  //   auto toc = std::chrono::steady_clock::now();

  //   met.success = true;
  //   met.opt_time_ms = std::chrono::duration<double, std::milli>(toc - tic).count();
  //   met.lbfgs_iters = 0;
  //   met.final_objective = spline_out.getJerk();   // 或你定义的代价
  //   met.max_omega = getMaxOmega(bsS);
  //   met.method = "bspline_init_order" + std::to_string(order) + "_M" + std::to_string(M);

  //   // 如果你的 violate_cost_ 已经改成吃 sampler，就直接：
  //   // violate_cost_.evaluate(bsS, ...); 并填 met.vio_*
  //   // 这里先保留为 0
  //   met.vio_p = 0; met.vio_v = 0; met.vio_a = 0; met.vio_j = 0;
  //   met.vio_d = 0; met.vio_l = 0; met.vio_c = 0; met.vio_omega = 0;

  //   appendMetricsToCsv(met, traj_csv_path_);

  //   return true;
  // }


} // namespace traj_opt