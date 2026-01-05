#include <traj_opt/traj_opt.h>
#include <traj_opt/traj_metrics.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt
{

  static Eigen::Vector3d car_p_, car_v_;
  static Eigen::Vector3d g_(0, 0, -9.8);
  static Trajectory init_traj_;
  static Trajectory bvp_traj;
  static bool initial_guess_ = false;

  static double tictoc_innerloop_;
  static double tictoc_integral_;

  static int iter_times_;

  //推力方向导数
  static Eigen::MatrixXd f_DN(const Eigen::Vector3d &x)
  {
    double x_norm_2 = x.squaredNorm();
    return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
  }

  //推力方向导数
  static Eigen::MatrixXd f_D2N(const Eigen::Vector3d &x, const Eigen::Vector3d &y)
  {
    double x_norm_2 = x.squaredNorm();
    double x_norm_3 = x_norm_2 * x.norm();
    Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
    return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
  }
  
  // SECTION  variables transformation and gradient transmission
  //max(x,0)的C2光滑化函数 
  static double smoothedL1(const double &x,
                           const double mu,
                           double &grad)
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
  
  static double smoothed01(const double &x,
                           double &grad)
  {
    static double mu = 0.01;
    static double mu4 = mu * mu * mu * mu;
    static double mu4_1 = 1.0 / mu4;
    if (x < -mu)
    {
      grad = 0;
      return 0;
    }
    else if (x < 0)
    {
      double y = x + mu;
      double y2 = y * y;
      grad = y2 * (mu - 2 * x) * mu4_1;
      return 0.5 * y2 * y * (mu - x) * mu4_1;
    }
    else if (x < mu)
    {
      double y = x - mu;
      double y2 = y * y;
      grad = y2 * (mu + 2 * x) * mu4_1;
      return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
    }
    else
    {
      grad = 0;
      return 1;
    }
  }

  template <typename EIGENVEC>  
  static void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT) {
    for (int i = 0; i < RT.size(); ++i) {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  static void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT) {
    for (int i = 0; i < VT.size(); ++i) {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) 
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  static inline double gdT2t(double t)
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
  static inline void dRealT_dVirtualT(
    const Eigen::VectorXd &RT,            // real durations (N)
    const EIGENVEC &VT,            // virtual vars   (N)
    const Eigen::VectorXd &gdRT,          // grad wrt RT    (N)
    EIGENVECGD &gdVT,                // grad wrt VT    (N) output
    double wei_time,                      // rhoT_ / wei_time_
    double &costT)                        // output
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

  // !SECTION variables transformation and gradient transmission

  // SECTION object function
  static inline double objectiveFunc(void *ptrObj,
                                     const double *x,
                                     double *grad,
                                     const int n)
  {
    iter_times_++;
    TrajOpt &obj = *(TrajOpt *)ptrObj;
    Eigen::Map<const Eigen::VectorXd> Virtual_T(x, obj.dim_t_);
    Eigen::Map<Eigen::VectorXd> grad_vt(grad, obj.dim_t_);
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad + obj.dim_t_, 3, obj.dim_p_);

    Eigen::VectorXd Dur_T(obj.N_);  
    VirtualT2RealT(Virtual_T, Dur_T);
    for (int i = 0; i < obj.N_; ++i) {
      if (Dur_T(i) < 1e-3) Dur_T(i) = 1e-3;
    }
    const double total_t = Dur_T.sum();

    Eigen::MatrixXd tailS(3, 4);
    tailS.col(0) = car_p_ + car_v_ * total_t;
    tailS.col(1) = car_v_;
    tailS.col(2).setZero();
    tailS.col(3).setZero();

    auto tic = std::chrono::steady_clock::now();
    obj.mincoOpt_.generate(obj.initS_, tailS, P, Dur_T);

    double cost = obj.mincoOpt_.getTrajSnapCost();
    obj.mincoOpt_.calGrads_CT();

    auto toc = std::chrono::steady_clock::now();
    tictoc_innerloop_ += (toc - tic).count();

    tic = std::chrono::steady_clock::now();
    obj.addTimeIntPenalty(cost);

    toc = std::chrono::steady_clock::now();
    tictoc_integral_ += (toc - tic).count();

    tic = std::chrono::steady_clock::now();
    obj.mincoOpt_.calGrads_PT();
    toc = std::chrono::steady_clock::now();
    tictoc_innerloop_ += (toc - tic).count();

    gradP = obj.mincoOpt_.gdP;
    double costT = 0;
    dRealT_dVirtualT(Dur_T, Virtual_T, obj.mincoOpt_.gdT, grad_vt, obj.rhoT_, costT);
    cost += costT;

    return cost;
  }


  // !SECTION object function
  static inline int earlyExit(void *ptrObj,
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
    TrajOpt &obj = *(TrajOpt *)ptrObj;
    if (obj.pause_debug_)
    {
      //TODO
    }
    // return k > 1e3;
    return 0;
  }

  static void    bvp(const double &t,
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

  static double getMaxOmega(Trajectory &traj)
  {
    double dt = 0.01;
    double max_omega = 0;
    for (double t = 0; t < traj.getTotalDuration(); t += dt)
    {
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d thrust = a - g_;
      Eigen::Vector3d zb_dot = f_DN(thrust) * j;
      double omega12 = zb_dot.norm();
      if (omega12 > max_omega)
      {
        max_omega = omega12;
      }
    }
    return max_omega;
  }

  static double getMaxVel(Trajectory &traj)
  {
    double dt = 0.01;
    double max_vel = 0;
    for (double t = 0; t < traj.getTotalDuration(); t += dt)
    {
      double v = traj.getVel(t).norm();
      if (v > max_vel)
      {
        max_vel = v;
      }
    }
    return max_vel;
  }

/**
 * @brief set landing parameters
*/
  void TrajOpt::setLandingParams(const LandingParams &lp) {
    is_landing_ = lp.is_landing;
    uwb_dist_ = lp.uwb_dist;
    if (is_landing_) {
      land_target_x_ = lp.land_x;
      land_target_y_ = lp.land_y;
      land_target_z_ = lp.land_z;
    }
  }

  bool TrajOpt::trans_bvp_traj(Trajectory &traj)
  {
    traj = bvp_traj;
    return true;
  }


  /*
  iniState: 初始状态 包含位置速度等信息
  car_p: 目标位置
  car_v: 目标速度
  land_q: 目标姿态
  N: 轨迹分段数
  t_replan: 重规划时间 t_replan默认为-1时，不进行重规划
  traj: 轨迹对象，用于存储生成的轨迹信息
  */
  bool TrajOpt::generate_traj(const Eigen::MatrixXd &iniState,
                              const Eigen::Vector3d &car_p,
                              const Eigen::Vector3d &car_v,
                              const Eigen::Quaterniond &land_q,
                              const int &N,
                              Trajectory &traj,
                              const double &t_replan)
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

    //initial_guess_初始为false
    //t_replan默认为-1，不进行重规划
    //opt_once一直为false？？
    bool opt_once = initial_guess_ && t_replan > 0 && t_replan < init_traj_.getTotalDuration();
    //如果存在初始猜测值
    if (opt_once) {
      //getDurations返回的是每段的时长
      Eigen::VectorXd durs = init_traj_.getDurations();
      if (durs.size() != N_) {
        // fallback：均分总时长 或 重新用BVP初始化
        Dur_T.setConstant(init_traj_.getTotalDuration() / N_);
      } else {
        Dur_T = durs;
      }
      RealT2VirtualT(Dur_T, Virtual_T);
      double t_abs = 0.0;
      for (int i = 1; i < N_; ++i) {
        t_abs += Dur_T(i-1);
        // getPos需要轨迹全局时间
        P.col(i - 1) = init_traj_.getPos(t_abs);
      }
    } else {
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

        bvp_traj = traj;
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
    }

    //----------------------
    //利用 BVP 解在均匀时间点采样，得到初始控制点 P，为后续优化提供合理的起点。

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
    tictoc_innerloop_ = 0;
    tictoc_integral_ = 0;
    iter_times_ = 0;

    //第一个参数 优化变量个数
    //minObjective 存储最小函数值的变量
    //objectiveFunc 计算目标函数和梯度的函数
    //earlyExit 满足某些条件时退出优化的函数
    //lbfgs_params 自定义参数
    opt_ret = lbfgs::lbfgs_optimize(dim_t_ + 3 * dim_p_, x_, &minObjective,
                                    &objectiveFunc, nullptr,
                                    &earlyExit, this, &lbfgs_params);

    //用于记录优化过程时间
    auto toc = std::chrono::steady_clock::now();

    if(opt_ret>=0) {
      // std::cout << "\033[32m>ret: " << opt_ret << "\033[0m" << std::endl;
    } else {
      auto err_msg = lbfgs::lbfgs_strerror(opt_ret);
      std::cout << "\033[31m>traj opt err: " << err_msg << "\033[0m" << std::endl;
    }

    if (pause_debug_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    //优化失败 清理内存并返回false
    if (opt_ret < 0) {
      delete[] x_;
      std::cout << "optimization failed" << std::endl;
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
    met.replan_t = t_replan;              // 记录是否是重规划
    met.method = "minco_lbfgs_piece" + std::to_string(N_);
    // met.scene_id = scene_id;           // 如果你有场景编号就填上
    met.vio_v = violate_cost_.cost_v_;
    met.vio_omega = violate_cost_.cost_omega_;
    //std::cout << "v cost: " << violate_cost_.cost_v_ << std::endl;     
    //std::cout << "omega cost: " << violate_cost_.cost_omega_ << std::endl;

    appendMetricsToCsv(met, "/home/lc/mc_simu_ws/traj_metrics.csv");
    init_traj_ = traj;
    initial_guess_ = true;
    delete[] x_;
    return true;
  }

  void TrajOpt::addTimeIntPenalty(double &cost)
  {
    Eigen::Vector3d pos, vel, acc, jer, snp;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    double cost_inner = 0.0;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::Matrix<double, 8, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
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

        if (!is_landing_ && feasibilityGradCostV(vel, gradv, violate_cost_.cost_v_)) {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaVc;
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_v_ / K_ + step * gradViolaVt);
          cost_inner += omg * step * violate_cost_.cost_v_;
        }

        if (feasibilityGradCostA(acc, grada, violate_cost_.cost_a_)) {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaAc;
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_a_ / K_ + step * gradViolaAt);
          cost_inner += omg * step * violate_cost_.cost_a_;
        }

        if (feasibilityGradCostJ(jer, gradj, violate_cost_.cost_j_)) {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snp;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaJc;
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_j_ / K_ + step * gradViolaJt);
          cost_inner += omg * step * violate_cost_.cost_j_;
        }

        if (feasibilityGradCostOmega(acc, jer, grada, gradj, violate_cost_.cost_omega_)) {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          gradViolaJt = alpha * gradj.transpose() * snp;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * (gradViolaAc + gradViolaJc);
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_omega_ / K_ + step * (gradViolaAt + gradViolaJt));
          cost_inner += omg * step * violate_cost_.cost_omega_;
        }

        if (!is_landing_ && EmerDistGradCostD(vel, gradv, violate_cost_.cost_d_)) {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaVc;
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_d_ / K_ + step * gradViolaVt);
          cost_inner += omg * step * violate_cost_.cost_d_;
        }

        if (is_landing_ && StrongWindAreaGradCostP(pos, gradp, violate_cost_.cost_p_)) {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaPc;
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_p_ / K_ + step * gradViolaPt);
          cost_inner += omg * step * violate_cost_.cost_p_;
        }

        if (is_landing_ && LandSmoothGradCost(pos, vel, gradv, violate_cost_.cost_lv_)) {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViolaVc;
          mincoOpt_.gdT(i) += omg * (violate_cost_.cost_lv_ / K_ + step * gradViolaVt);
          cost_inner += omg * step * violate_cost_.cost_lv_;
        }

        s1 += step;
      }
    }
    cost += cost_inner;
  }


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
    nh.param("traj_opt/rhoOmega", rhoOmega_, 100000.0);
    nh.param("traj_opt/rhoLV", rho_LV_, 100000.0);
    nh.param("traj_opt/LV_max", LV_max_, 0.5);
    nh.param("traj_opt/LV_min", LV_min_, 0.1);
    nh.param("traj_opt/emergency_stop_dist", emergency_stop_dist_, 1.0);
    nh.param("traj_opt/safe_aera_radius", safe_aera_radius_, 0.2);
    nh.param("traj_opt/pause_debug", pause_debug_, false);
    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
  }

  bool TrajOpt::StrongWindAreaGradCostP(const Eigen::Vector3d &p, Eigen::Vector3d &gradp, double &costp) {
    constexpr double mu = 0.01;
    double ppenx = std::fabs(p.x() - land_target_x_) - safe_aera_radius_;
    double ppeny = std::fabs(p.y() - land_target_y_) - safe_aera_radius_;

    if (ppenx < 0 && ppeny < 0) {
      gradp.setZero();
      costp = 0.0;
      return false;
    }

    if (ppenx > 0) {
      double dx = 0.0;
      costp += rhoP_ * smoothedL1(ppenx, mu, dx);
      gradp.x() = rhoP_ * dx;
    }

    if (ppeny > 0) {
      double dy = 0.0;
      costp += rhoP_ * smoothedL1(ppeny, mu, dy);
      gradp.y() = rhoP_ * dy;
    }

    return true;
  }

  bool TrajOpt::feasibilityGradCostV(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costv) {
    constexpr double mu = 0.01;
    double vpen = v.squaredNorm() - vmax_ * vmax_;

    if (vpen  < 0) {
      gradv.setZero();
      costv = 0.0;
      return false;
    }

    double d = 0.0;
    costv = rhoV_ * smoothedL1(vpen, mu, d);
    gradv = rhoV_ * 2 * d * v;
    return true;
    // gradv = rhoV_ * 6 * vpen * vpen * v;
    // costv = rhoV_ * vpen * vpen * vpen;
    // return true;
  }

  bool TrajOpt::feasibilityGradCostA(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa) {
    constexpr double mu = 0.01;
    double apen = a.squaredNorm() - amax_ * amax_;

    if (apen  < 0) {
      grada.setZero();
      costa = 0.0;
      return false;
    }

    double d = 0.0;
    costa = rhoA_ * smoothedL1(apen, mu, d);
    grada = rhoA_ * 2 * d * a;
    return true;
    // grada = rhoA_ * 6 * apen * apen * a;
    // costa = rhoA_ * apen * apen * apen;
    // return true;
  }

  bool TrajOpt::feasibilityGradCostJ(const Eigen::Vector3d &j, Eigen::Vector3d &gradj, double &costj) {
    constexpr double mu = 0.01;
    double jpen = j.squaredNorm() - jmax_ * jmax_;

    if (jpen  < 0) {
      gradj.setZero();
      costj = 0.0;
      return false;
    }

    double d = 0.0;
    costj = rhoJ_ * smoothedL1(jpen, mu, d);
    gradj = rhoJ_ * 2 * d * j;
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
    constexpr double mu = 0.01;
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

  bool TrajOpt::EmerDistGradCostD(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costd) {
    constexpr double mu = 0.01;
    double dpen = uwb_dist_ - emergency_stop_dist_;

    if (dpen > 0) {
      gradv.setZero();
      costd = 0.0;
      return false;
    }

    double d = 0.0;
    costd = rho_D_ * v.squaredNorm() * smoothedL1(std::fabs(dpen), mu, d);
    gradv = rho_D_ * 2 * v * d;
    return true;
  }

  bool TrajOpt::LandSmoothGradCost(const Eigen::Vector3d &p, const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costlv) {
    constexpr double mu = 0.01;
    double delta_z = p.z() - land_target_z_;

    double allowed_vmax = computeAllowedVmaxLV(delta_z);
    double lvpen = v.squaredNorm() - allowed_vmax;

    if (lvpen < 0) {
      gradv.setZero();
      costlv = 0.0;
      return false;
    }

    double d = 0.0;
    double penalty_weight = computePenaltyWeightLV(delta_z);
    costlv = penalty_weight * smoothedL1(lvpen, mu, d);
    gradv = penalty_weight * 2 * v * d;
    return true;
  }

  double TrajOpt::computeAllowedVmaxLV(double delta_z) {
      // 方案1：线性衰减
      // return LV_min_ + (LV_max_ - LV_min_) * 
      //        (delta_z / (delta_z + 1.0));  // 饱和函数
      
      // 方案2：指数衰减（更平滑）
      double ratio = delta_z / (delta_z + 1.0);
      return LV_min_ + (LV_max_ - LV_min_) * 
              (1.0 - exp(-3.0 * ratio));
      
      // 方案3：分段线性
      // if (delta_z > 2.0) return LV_max_;
      // else if (delta_z > 1.0) return LV_min_ + (LV_max_ - LV_min_) * (delta_z - 1.0) / 4.0;
      // else return LV_min_ + (0.5 - LV_min_) * delta_z;
  }
  
  double TrajOpt::computePenaltyWeightLV(double delta_z) {
      // 使用反比例函数：高度越低，惩罚越强
      const double base_weight = rho_LV_;
      return base_weight * (1.0 + 5.0 / (delta_z + 0.1));
  }


} // namespace traj_opt