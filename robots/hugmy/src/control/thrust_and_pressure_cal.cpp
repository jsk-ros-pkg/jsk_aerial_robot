#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>
#include <limits>


class ThetaModel{
public:
  static constexpr int POLY_DEGREE = 2;
  std::vector<double> P_grid;
  std::vector<double> T_grid;
  std::vector<std::vector<double>> theta_grid;
  Eigen::VectorXd coeffs;

  void buildSurface() {
    const int nP = (int)P_grid.size();
    const int nT = (int)T_grid.size();
    fx_.assign(nP, std::vector<double>(nT, 0.0));
    fy_.assign(nP, std::vector<double>(nT, 0.0));
    fxy_.assign(nP, std::vector<double>(nT, 0.0));

    // fx (∂θ/∂P)
    for (int i = 0; i < nP; ++i) {
      for (int j = 0; j < nT; ++j) {
        if (i == 0) {
          double dP = P_grid[i+1] - P_grid[i];
          fx_[i][j] = (theta_grid[i+1][j] - theta_grid[i][j]) / dP;
        } else if (i == nP-1) {
          double dP = P_grid[i] - P_grid[i-1];
          fx_[i][j] = (theta_grid[i][j] - theta_grid[i-1][j]) / dP;
        } else {
          double dP = P_grid[i+1] - P_grid[i-1];
          fx_[i][j] = (theta_grid[i+1][j] - theta_grid[i-1][j]) / dP;
        }
      }
    }
    // fy (∂θ/∂T)
    for (int i = 0; i < nP; ++i) {
      for (int j = 0; j < nT; ++j) {
        if (j == 0) {
          double dT = T_grid[j+1] - T_grid[j];
          fy_[i][j] = (theta_grid[i][j+1] - theta_grid[i][j]) / dT;
        } else if (j == nT-1) {
          double dT = T_grid[j] - T_grid[j-1];
          fy_[i][j] = (theta_grid[i][j] - theta_grid[i][j-1]) / dT;
        } else {
          double dT = T_grid[j+1] - T_grid[j-1];
          fy_[i][j] = (theta_grid[i][j+1] - theta_grid[i][j-1]) / dT;
        }
      }
    }
    // fxy (∂²θ/∂P∂T) ー 交差中央差分
    for (int i = 0; i < nP; ++i) {
      for (int j = 0; j < nT; ++j) {
        int i0 = std::max(0, i-1), i1 = std::min(nP-1, i+1);
        int j0 = std::max(0, j-1), j1 = std::min(nT-1, j+1);
        double dP = P_grid[i1] - P_grid[i0];
        double dT = T_grid[j1] - T_grid[j0];
        if (dP > 0 && dT > 0) {
          fxy_[i][j] = (theta_grid[i1][j1] - theta_grid[i1][j0]
                       -theta_grid[i0][j1] + theta_grid[i0][j0]) / (dP * dT);
        } else {
          fxy_[i][j] = 0.0;
        }
      }
    }
    built_ = true;
  }

  // Bicubic Hermite 補間で θ を評価（範囲外は端にクランプ）
  double f_theta_bicubic(double P, double T) const {
    if (!built_) return f_theta_bilinear(P, T); // フォールバック

    // 1) クランプ & セルを見つける
    P = std::max(P_grid.front(), std::min(P, P_grid.back()));
    T = std::max(T_grid.front(), std::min(T, T_grid.back()));

    int i = std::max(0, (int)(std::lower_bound(P_grid.begin(), P_grid.end(), P) - P_grid.begin()) - 1);
    int j = std::max(0, (int)(std::lower_bound(T_grid.begin(), T_grid.end(), T) - T_grid.begin()) - 1);
    if (i >= (int)P_grid.size()-1) i = (int)P_grid.size()-2;
    if (j >= (int)T_grid.size()-1) j = (int)T_grid.size()-2;

    double P0 = P_grid[i],   P1 = P_grid[i+1];
    double T0 = T_grid[j],   T1 = T_grid[j+1];
    double dx = P1 - P0,     dy = T1 - T0;
    double u = (dx > 0) ? (P - P0) / dx : 0.0;   // [0,1]
    double v = (dy > 0) ? (T - T0) / dy : 0.0;   // [0,1]

    double f00 = theta_grid[i  ][j  ], f10 = theta_grid[i+1][j  ];
    double f01 = theta_grid[i  ][j+1], f11 = theta_grid[i+1][j+1];
    double fx00 = fx_[i  ][j  ], fx10 = fx_[i+1][j  ];
    double fx01 = fx_[i  ][j+1], fx11 = fx_[i+1][j+1];
    double fy00 = fy_[i  ][j  ], fy10 = fy_[i+1][j  ];
    double fy01 = fy_[i  ][j+1], fy11 = fy_[i+1][j+1];
    double fxy00 = fxy_[i  ][j  ], fxy10 = fxy_[i+1][j  ];
    double fxy01 = fxy_[i  ][j+1], fxy11 = fxy_[i+1][j+1];

    // 3) 1D エルミート基底
    auto h00 = [](double t){ return  2*t*t*t - 3*t*t + 1; };
    auto h10 = [](double t){ return      t*t*t - 2*t*t + t; };
    auto h01 = [](double t){ return -2*t*t*t + 3*t*t; };
    auto h11 = [](double t){ return      t*t*t -   t*t; };

    // 4) テンソル積（注意: 偏導はスケーリングが必要）
    double val =
      h00(u)*h00(v)*f00 + h01(u)*h00(v)*f10 + h00(u)*h01(v)*f01 + h01(u)*h01(v)*f11
    + h10(u)*h00(v)*(fx00*dx) + h11(u)*h00(v)*(fx10*dx) + h10(u)*h01(v)*(fx01*dx) + h11(u)*h01(v)*(fx11*dx)
    + h00(u)*h10(v)*(fy00*dy) + h01(u)*h10(v)*(fy10*dy) + h00(u)*h11(v)*(fy01*dy) + h01(u)*h11(v)*(fy11*dy)
    + h10(u)*h10(v)*(fxy00*dx*dy) + h11(u)*h10(v)*(fxy10*dx*dy)
    + h10(u)*h11(v)*(fxy01*dx*dy) + h11(u)*h11(v)*(fxy11*dx*dy);

    return val;
  }

  // 補間面の上での P 逆写像（θd,T固定→P）
  double inverse_P_on_surface(double theta_d, double T_nom) const {
    // 1) T をクランプ
    double T = std::max(T_grid.front(), std::min(T_nom, T_grid.back()));

    // 2) P 範囲で θ(P,T) の端を見て到達可能域チェック
    double th_lo = f_theta_bicubic(P_grid.front(), T);
    double th_hi = f_theta_bicubic(P_grid.back(),  T);
    // 端を含む単調でない場合も想定し、区分ごとに交差探索
    // 3) セルごとに探索（P の隣接グリッドで符号反転を探す）
    for (int i = 1; i < (int)P_grid.size(); ++i) {
      double Pa = P_grid[i-1], Pb = P_grid[i];
      double fa = f_theta_bicubic(Pa, T) - theta_d;
      double fb = f_theta_bicubic(Pb, T) - theta_d;
      if (fa == 0.0) return Pa;
      if (fb == 0.0) return Pb;
      if ((fa > 0 && fb < 0) || (fa < 0 && fb > 0)) {
        // 4) 区間 [Pa,Pb] で二分法
        double a = Pa, b = Pb;
        for (int it=0; it<40; ++it) {
          double m = 0.5*(a+b);
          double fm = f_theta_bicubic(m, T) - theta_d;
          if (std::abs(fm) < 1e-6 || std::abs(b-a) < 1e-6) return m;
          if ((fa > 0 && fm < 0) || (fa < 0 && fm > 0)) { b = m; fb = fm; }
          else { a = m; fa = fm; }
        }
        return 0.5*(a+b);
      }
    }
    // 交差なし→端にクランプ
    double d_lo = std::abs(th_lo - theta_d);
    double d_hi = std::abs(th_hi - theta_d);
    return (d_lo <= d_hi) ? P_grid.front() : P_grid.back();
  }

  double f_theta_bilinear(double P, double T) const {
    P = std::max(P_grid.front(), std::min(P, P_grid.back()));
    T = std::max(T_grid.front(), std::min(T, T_grid.back()));
    auto itP = std::lower_bound(P_grid.begin(), P_grid.end(), P);
    auto itT = std::lower_bound(T_grid.begin(), T_grid.end(), T);
    int i = std::max(0, (int)(itP - P_grid.begin()) - 1);
    int j = std::max(0, (int)(itT - T_grid.begin()) - 1);
    if (i >= (int)P_grid.size()-1) i = (int)P_grid.size()-2;
    if (j >= (int)T_grid.size()-1) j = (int)T_grid.size()-2;
    double P0=P_grid[i], P1=P_grid[i+1], T0=T_grid[j], T1=T_grid[j+1];
    double u = (P1>P0) ? (P-P0)/(P1-P0) : 0.0;
    double v = (T1>T0) ? (T-T0)/(T1-T0) : 0.0;
    double q00=theta_grid[i][j], q10=theta_grid[i+1][j];
    double q01=theta_grid[i][j+1], q11=theta_grid[i+1][j+1];
    double q0 = (1-u)*q00 + u*q10;
    double q1 = (1-u)*q01 + u*q11;
    return (1-v)*q0 + v*q1;
  }

  static Eigen::VectorXd polyFeatures(double P, double T, int degree=POLY_DEGREE) {
    std::vector<double> feats;
    for (int i = 0; i <= degree; ++i)
        for (int j = 0; j <= degree - i; ++j)
            feats.push_back(std::pow(P,i) * std::pow(T,j));
    Eigen::VectorXd v(feats.size());
    for(size_t k=0;k<feats.size();++k) v(k)=feats[k];
    return v;
  }

  void fit(int degree=POLY_DEGREE) {
    int nP=P_grid.size(), nT=T_grid.size();
    int nSamples = nP * nT;
    int nFeatures = (degree+1)*(degree+2)/2;
    Eigen::MatrixXd X(nSamples, nFeatures);
    Eigen::VectorXd y(nSamples);
    int idx=0;
    for(int i=0;i<nP;i++){
      for(int j=0;j<nT;j++){
        X.row(idx)=polyFeatures(P_grid[i], T_grid[j], degree).transpose();
        y(idx)=theta_grid[i][j];
        idx++;
      }
    }
    coeffs = (X.transpose()*X).ldlt().solve(X.transpose()*y);
  }

  double f_theta(double P, double T) const {
    return polyFeatures(P,T).dot(coeffs);
  }

  std::pair<double,double> jacobian(double P,double T,double dP=1.0,double dT=0.01) const {
    double thPp=f_theta(P+dP,T), thPm=f_theta(P-dP,T);
    double thTp=f_theta(P,T+dT), thTm=f_theta(P,T-dT);
    return {(thPp-thPm)/(2*dP),(thTp-thTm)/(2*dT)};
  }

  double inverse_T_ff(double theta_d,double P_nom) const {
    std::vector<double> ths;
    for(auto T: T_grid) ths.push_back(f_theta(P_nom,T));
    for(size_t i=1;i<T_grid.size();++i){
      double th0=ths[i-1], th1=ths[i];
      if((theta_d>=th0 && theta_d<=th1)||(theta_d<=th0 && theta_d>=th1)){
        double t=(theta_d-th0)/(th1-th0);
        return T_grid[i-1]+t*(T_grid[i]-T_grid[i-1]);
      }
    }
    if(theta_d<ths.front()) return T_grid.front();
    if(theta_d>ths.back()) return T_grid.back();
    throw std::runtime_error("inverse failed");
  }

  double inverse_P_ff(double theta_d,double T_nom) const {
    std::vector<double> ths;
    ths.reserve(P_grid.size());
    for(auto P: P_grid) ths.push_back(f_theta(P,T_nom));
    for(size_t i=1;i<P_grid.size();++i){
      double th0=ths[i-1], th1=ths[i];
      if((theta_d>=th0 && theta_d<=th1)||(theta_d<=th0 && theta_d>=th1)){
        double t=(theta_d-th0)/(th1-th0);
        return P_grid[i-1]+t*(P_grid[i]-P_grid[i-1]);
      }
    }
    if(theta_d<ths.front()) return P_grid.front();
    if(theta_d>ths.back()) return P_grid.back();
    throw std::runtime_error("inverse failed");
  }

  double solve_P_for_theta_bisection(double theta_target,
                                   double T_fixed,
                                   int max_iter = 50,
                                   double tol = 1e-4) const
  {

    auto g = [&](double P){ return f_theta(P, T_fixed) - theta_target; };


    for (size_t i = 1; i < P_grid.size(); ++i) {
      double a = P_grid[i-1], b = P_grid[i];
      double ga = g(a), gb = g(b);
      if (std::isnan(ga) || std::isnan(gb)) continue;

      if (ga == 0.0) return a;
      if (gb == 0.0) return b;

      if ((ga > 0 && gb < 0) || (ga < 0 && gb > 0)) {

        for (int it = 0; it < max_iter; ++it) {
          double m = 0.5 * (a + b);
          double gm = g(m);
          if (std::abs(gm) < tol || std::abs(b - a) < tol) return m;

          if ((ga > 0 && gm < 0) || (ga < 0 && gm > 0)) {
            b = m; gb = gm;
          } else {
            a = m; ga = gm;
          }
        }
        return 0.5 * (a + b);
      }
    }

    double g_lo = g(P_grid.front());
    double g_hi = g(P_grid.back());
    double th_lo = f_theta(P_grid.front(), T_fixed);
    double th_hi = f_theta(P_grid.back(),  T_fixed);
    double d_lo = std::abs(th_lo - theta_target);
    double d_hi = std::abs(th_hi - theta_target);
    return (d_lo <= d_hi) ? P_grid.front() : P_grid.back();
  }

  std::pair<double,double> newton_find_PT_for_target(double theta_target,
                                                     double P_init = 30.0,
                                                     double T_init = 2.5,
                                                     int max_iter = 20,
                                                     double tol = 1e-4,
                                                     double alpha = 0.3)
  {
    double P = P_init, T = T_init;
    for(int iter=0; iter<max_iter; ++iter){
      double theta=f_theta(P,T);
      auto [dthdP,dthdT]=jacobian(P,T);
      double err = theta - theta_target;
      if(std::abs(err)<tol) { std::cout<<"Converged at iter "<<iter<<"\n"; break; }

      Eigen::Vector2d grad(dthdP,dthdT);
      if(grad.squaredNorm()<1e-8){ std::cerr<<"Jacobian too small\n"; break; }

      Eigen::Vector2d delta = alpha * (grad / grad.squaredNorm()) * err;
      P -= delta(0);  T -= delta(1);
      P = std::clamp(P, 0.0, 50.0);
      T = std::clamp(T, 0.0, 7.0);

      std::cout<<"iter "<<iter<<"  P="<<P<<"  T="<<T
               <<"  θ="<<theta<<"  err="<<err<<"\n";
    }
    return {P,T};
  }
private:
  bool built_ = false;
  std::vector<std::vector<double>> fx_, fy_, fxy_;

};

class ThrustPressureCal{
public:
  ThrustPressureCal() : nh_("~")
  {
    theta_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/theta", 1, &ThrustPressureCal::targetThetaCallback, this);
    theta_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/debug/arm/target_theta", 1);
    // thrust_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/debug/arm/thrust_ref", 1);
    thrust_cur_sub_ = nh_.subscribe<spinal::FourAxisCommand>("/quadrotor/four_axes/command", 1, &ThrustPressureCal::targetThrustCallback, this);
    // effort_pub_  = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joints_ctrl",1);
    // effort_cur_sub_ = nh_.subscribe<sensor_msgs::JointState>("/quadrotor/joint_states", 1, &ThrustPressureCal::servoEffortCallback, this);
    pressure_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/arm/pressure_cmd", 1);

    nh_.param("theta_target", theta_target_, 0.0);
    nh_.param("tau_limit", tau_limit_, 2.0);
    nh_.param("current_pressure", current_pressure_, 2.0);
    nh_.param("lpf_alpha", lpf_alpha_, 1.0); // 1.0 で LPFなし


    // 係数
    nh_.param("k2", k2_, -1.457);
    nh_.param("k1", k1_, 0.1745);
    nh_.param("k0", k0_, 0.2206);

    // LPF 初期化
    tau_lpf_.assign(12, 0.0);

    thrust_lpf_ = 0.0f;

    initModel();
    model_.buildSurface();
  }

  void spin(){
    ros::Rate rate(100);
    while(ros::ok()){
      // nh_.getParam("theta_target", theta_target_);
      // nh_.getParam("current_pressure", current_pressure_);
 
      // auto [P_opt, T_opt] = model_.newton_find_PT_for_target(theta_target_, current_pressure_, thrust_cur_avr_ + 1.0);
      // std::cout << "Thrust_current = " << thrust_cur_avr_ << "\n";
      // int P_int = static_cast<int> (std::round(P_opt));
      // std::cout << "Result: P= " << P_int << "kPa, T = " << T_opt
      //           <<"N, theta" << model_.f_theta(P_opt, T_opt) << "deg\n";
      // double theta_est = model_.f_theta(P_opt, T_opt);


      // std::vector<double> P_each;
      // P_each.reserve(thrust_cur_.size());
      // for (float T_i : thrust_cur_) {
      //   // double P_i = model_.solve_P_for_theta_bisection(theta_target_, T_i);
      //   double P_i = model_.inverse_P_on_surface(theta_target_, T_i);
      //   P_each.push_back(P_i);
      // }

      const double T_nom = thrust_cur_avr_;
      double P_cmd = model_.inverse_P_on_surface(theta_target_, T_nom);
      // if (!P_each.empty()){
      //   P_cmd = std::accumulate(P_each.begin(), P_each.end(), 0.0) / P_each.size();
      // }

      double theta_est = model_.f_theta_bicubic(P_cmd, T_nom);

      std_msgs::Float32 msg_theta, msg_thrust, msg_pressure;
      msg_theta.data = theta_est;
      msg_thrust.data = thrust_cur_avr_;
      msg_pressure.data = static_cast<float>(P_cmd);
      theta_pub_.publish(msg_theta);
      // thrust_pub_.publish(msg_thrust);
      pressure_pub_.publish(msg_pressure);
      
      // ROS_INFO_STREAM("Check table corners via bicubic: "
      //                 << " (P=0,T=0)="   << model_.f_theta_bicubic(0, 0)
      //                 << " (P=10,T=5)="   << model_.f_theta_bicubic(10, 5)
      //                 << " (P=50,T=0)="  << model_.f_theta_bicubic(50, 0)
      //                 << " (P=50,T=7)="  << model_.f_theta_bicubic(50, 7));
      
      ROS_INFO_STREAM_THROTTLE(0.5,
        "theta_target=" << theta_target_
        << "  T_target=" << thrust_cur_avr_
        << "  -> P_cmd=" << P_cmd
        << "  (theta_est=" << theta_est << ")"
      );


      // sensor_msgs::JointState js;
      // js.header.stamp = ros::Time::now();
      // js.name.reserve(12);
      // js.position.reserve(12);
      
      // std_msgs::Float64MultiArray tau_cmd;
      // tau_cmd.data.resize(12);
      // for (int i = 0; i < tau_cmd.data.size(); ++i){
      //   double tau = 3.888e-6 * (k0_ * P_opt - k1_ * P_opt * theta_target_) - 5.054e-8 * k2_ * P_opt;
      //   if (tau > tau_limit_) tau = tau_limit_;
      //   if (tau < - tau_limit_) tau = - tau_limit_;
      //   tau_lpf_[i] = (1.0 - lpf_alpha_) * tau_lpf_[i] + lpf_alpha_ * tau;
      //   tau_cmd.data[i] = (effort_cur_[i] + tau_lpf_[i]);
      // }
      // js.position = tau_cmd.data;

      // for (int id = 1; id <= 4; ++id)
      //   {
      //     for (int j = 1; j <= 3; ++j){
      //       std::string js_name = "joint_" +  std::to_string(j) + "_" + std::to_string(id);
      //       js.name.push_back(js_name);
      //     }
      //   }
      // ROS_INFO_STREAM_THROTTLE(0.5, "js" << js);
      // effort_pub_.publish(js);

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher theta_pub_, pressure_pub_;
  ros::Subscriber theta_sub_;
  // ros::Publisher effort_pub_;

  // ros::Subscriber effort_cur_sub_;
  ros::Subscriber thrust_cur_sub_;
  ThetaModel model_;

  double current_pressure_{0.0};
  double tau_limit_{2.0};
  double lpf_alpha_{1.0};
  double k2_{-1.457}, k1_{0.1745}, k0_{0.2206};
  std::vector<double> tau_lpf_;
  double last_P_opt_{0.0};

  double theta_target_;
  float thrust_cur_avr_ = 7.0;
  float thrust_lpf_ = 0.0f;
  std::vector<float> thrust_cur_;
  // std::vector<double> effort_cur_;
  sensor_msgs::JointState pub_msg_;

  void initModel() {
    model_.P_grid = {0,5,10,15,20,25,30,35,40,45,50};
    model_.T_grid = {0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7};
    model_.theta_grid = {
        {139.768,134.493,128.551,124.460,119.977,113.315,101.055,89.273,63.946,3.289,9.258,0.227,0.472,0.430,0},
        {144.643,139.467,137.555,132.286,128.972,121.224,111.347,104.471,89.099,42.255,46.203,1.461,1.736,0.496,0.549},
        {147.598,145.220,141.971,137.005,132.727,125.932,123.080,115.013,100.787,33.402,57.619,1.770,5.143,2.577,3.129},
        {152.154,149.182,146.960,141.890,138.102,131.389,126.483,120.379,110.474,93.181,92.292,31.991,13.342,11.450,7.433},
        {157.131,156.080,152.754,146.997,143.930,137.597,131.136,127.663,120.223,104.781,98.179,43.062,43.385,30.329,27.754},
        {161.013,158.571,155.689,153.529,147.594,143.465,139.130,133.446,125.879,124.570,105.655,86.253,53.938,46.389,41.418},
        {164.280,162.752,160.557,155.001,153.065,148.041,143.542,138.050,132.759,175.925,116.943,98.069,93.442,92.621,56.854},
        {167.100,164.741,163.243,158.070,155.979,152.618,147.705,141.981,133.668,176.184,123.087,111.130,106.004,104.060,93.947},
        {168.751,168.114,163.771,162.540,159.212,156.223,153.237,148.180,141.759,129.246,128.565,105.031,114.042,109.748,104.201},
        {170.385,169.522,166.386,163.503,163.294,160.484,156.870,152.493,145.866,133.232,135.627,111.236,172.964,115.230,115.908},
        {174.290,166.915,170.521,169.391,166.849,163.922,159.505,158.103,151.329,139.333,143.576,116.591,126.471,125.471,122.980}
    };
    // model_.fit();
    model_.buildSurface();
    // effort_cur_.resize(20);
  }

  void targetThrustCallback(const spinal::FourAxisCommand::ConstPtr& msg){
    if (!msg || msg -> base_thrust.empty()) return;
    thrust_cur_ = msg -> base_thrust;
    // std::cout << "thrust_current:" <<  thrust_cur_.at(0) <<"\n";
    double thrust_sum = 0;
    for(size_t i=0; i < thrust_cur_.size(); ++i){
      thrust_sum += thrust_cur_[i];
    }
    thrust_cur_avr_ = (thrust_sum / static_cast<float>(thrust_cur_.size())) + 1.8;
    // std::cout << "thrust_current_avr:" <<  thrust_cur_avr_ <<"\n";
  }

  void targetThetaCallback(const std_msgs::Float32::ConstPtr& msg){
    theta_target_ = msg -> data;
  }

  // void servoEffortCallback(const sensor_msgs::JointState::ConstPtr& msg){
  //   effort_cur_ = msg -> effort;
  // }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "thrust_and_pressure_cal");
  ThrustPressureCal node;
  node.spin();
  return 0;
}
