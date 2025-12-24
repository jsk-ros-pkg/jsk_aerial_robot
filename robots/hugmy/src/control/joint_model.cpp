#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <std_msgs/Float32.h>
// #include <spinal/Thrust.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

class ThetaModel{
public:
  static constexpr int POLY_DEGREE = 2;
  std::vector<double> P_grid;
  std::vector<double> T_grid;
  std::vector<std::vector<double>> theta_grid;
  Eigen::VectorXd coeffs;

  // static Eigen::VectorXd polyFeatures(double P, double T, int degree=POLY_DEGREE) {
  //   std::vector<double> feats;
  //   for (int i = 0; i <= degree; ++i)
  //     for (int j = 0; j <= degree - i; ++j)
  //       feats.push_back(std::pow(P,i) * std::pow(T,j));
  //   Eigen::VectorXd v(feats.size());
  //   for(size_t k=0;k<feats.size();++k) v(k)=feats[k];
  //   return v;
  // }

  // void fit(int degree=POLY_DEGREE) {
  //   const int nP = static_cast<int>(P_grid.size());
  //   const int nT = static_cast<int>(T_grid.size());
  //   const int nSamples  = nP * nT;
  //   const int nFeatures = (degree+1)*(degree+2)/2;

  //   Eigen::MatrixXd X(nSamples, nFeatures);
  //   Eigen::VectorXd y(nSamples);

  //   int idx=0;
  //   for(int i=0;i<nP;i++){
  //     for(int j=0;j<nT;j++){
  //       X.row(idx) = polyFeatures(P_grid[i], T_grid[j], degree).transpose();
  //       y(idx)     = theta_grid[i][j];
  //       ++idx;
  //     }
  //   }
  //   coeffs = (X.transpose()*X).ldlt().solve(X.transpose()*y);
  // }


  // double f_theta(double P, double T) const {
  //   return polyFeatures(P,T).dot(coeffs);
  // }

  void fit(){}
  double f_theta(double P, double T) const {
    P = std::max(P_grid.front(), std::min(P, P_grid.back()));
    T = std::max(T_grid.front(), std::min(T, T_grid.back()));

    auto itP = std::lower_bound(P_grid.begin(), P_grid.end(), P);
    auto itT = std::lower_bound(T_grid.begin(), T_grid.end(), T);

    int j = std::distance(P_grid.begin(), itP);
    int k = std::distance(T_grid.begin(), itT);

    if (j < (int)P_grid.size() && P_grid[j] == P && k < (int)T_grid.size() && T_grid[k] == T)
      return theta_grid[j][k];

    int j0 = std::max(0, j - 1);
    int j1 = std::min(j0 + 1, (int)P_grid.size() - 1);
    int k0 = std::max(0, k - 1);
    int k1 = std::min(k0 + 1, (int)T_grid.size() - 1);

    double P0 = P_grid[j0], P1 = P_grid[j1];
    double T0 = T_grid[k0], T1 = T_grid[k1];

    double tP = (P1 > P0) ? (P - P0) / (P1 - P0) : 0.0;
    double tT = (T1 > T0) ? (T - T0) / (T1 - T0) : 0.0;

    double q00 = theta_grid[j0][k0];
    double q01 = theta_grid[j0][k1];
    double q10 = theta_grid[j1][k0];
    double q11 = theta_grid[j1][k1];

    double q0 = (1 - tP) * q00 + tP * q10;
    double q1 = (1 - tP) * q01 + tP * q11;
    return (1 - tT) * q0 + tT * q1;
  }

  std::pair<std::pair<double,double>, std::pair<double,double>> ranges() const {
    return {{P_grid.front(), P_grid.back()}, {T_grid.front(), T_grid.back()}};
  }
};


class JointModel{
public:
  JointModel() : nh_("~")
  {
    theta_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/quadrotor/debug/theta", 1);
    apply_to_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joint_states", 1);
    pressure_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/filterd_joint_cur_pressure", 1, &JointModel::pressureCb, this);
    //sim
    // pressure_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/sim_pressure", 1, &JointModel::pressureCb, this);
    thrust_sub_   = nh_.subscribe<spinal::FourAxisCommand>("/quadrotor/four_axes/command", 1, &JointModel::thrustCb, this);
    mode_sub_   = nh_.subscribe<std_msgs::Empty>("/quadrotor/arm/joint_estimate_enable", 1, &JointModel::modeCb, this);
    apply_to_joint_sim_pub_ = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joints_ctrl", 1);

    // nh_.param("pressure", latest_pressure_, 0.0f); // kPa
    // nh_.param("thrust",   latest_thrust_,   f);  // N

    initModel();

    latest_thrust_.assign(4, 7.0f);
    theta_deg_.assign(4, 0.0f);
    theta_rad_.assign(4, 0.0f);

    theta_rad_filt_.assign(4, 0.0f);
    nh_.param("theta_lpf_tau", lpf_tau_sec_, 0.20);
    nh_.param("theta_max_slew", max_delta_rad_per_sec_, 2.0);
    last_update_ = ros::Time::now();


    ROS_INFO_STREAM("ThrustPressureToTheta: pressure=" << latest_pressure_
                    << ", thrust=" << latest_thrust_.at(0)
                    << ", theta_out=" << theta_deg_.at(0) );
  }

  void spin(){
    ros::Rate rate(100);
    while(ros::ok()){

      ros::Time now = ros::Time::now();
      double dt = (now - last_update_).toSec();
      if (dt <= 0.0) dt =1e-3;
      last_update_ = now;
      //   nh_.getParam("pressure", latest_pressure_);
      //   nh_.getParam("thrust",   latest_thrust_);

      const double tau_p = std::max(1e-3, pressure_lpf_sec_);
      const double alpha_p = dt / (tau_p + dt);

      if (!pressure_filt_inited_) {
        latest_pressure_filt_ = latest_pressure_;
        pressure_filt_inited_ = true;
      } else {
        double prev_p = static_cast<double>(latest_pressure_filt_);
        double raw_p = static_cast<double>(latest_pressure_);
        double lpf_out_p = prev_p + alpha_p * (raw_p - prev_p);

        double max_step_p = max_pressure_per_sec_ * dt;
        double delta_p = lpf_out_p - prev_p;
        if (delta_p >  max_step_p) lpf_out_p = prev_p + max_step_p;
        if (delta_p < -max_step_p) lpf_out_p = prev_p - max_step_p;
        latest_pressure_filt_ = static_cast<float>(lpf_out_p);
      }

      int P_int = static_cast<int>(latest_pressure_filt_);
      double P = static_cast<double>(P_int);

      // ROS_WARN_STREAM_THROTTLE(1.0, "latest_pressure_ :" << latest_pressure_);
      for(int i = 0; i < 4; ++i){
        double T = static_cast<double>(latest_thrust_.at(i));

        auto [Pr, Tr] = model_.ranges();
        bool clamped = false;
        if (P < Pr.first)  { P = Pr.first;  clamped = true; }
        if (P > Pr.second) { P = Pr.second; clamped = true; }
        if (T < Tr.first)  { T = Tr.first;  clamped = true; }
        if (T > Tr.second) { T = Tr.second; clamped = true; }
        if (clamped){
          ROS_WARN_THROTTLE(1.0, "Input out of range. Clamped to P=[%.1f, %.1f], T=[%.1f, %.1f].",
                            Pr.first, Pr.second, Tr.first, Tr.second);
        }
        const double theta_est_deg = model_.f_theta(P, T);
        theta_deg_[i] = static_cast<float>(theta_est_deg);
        theta_rad_[i] = static_cast<float>(theta_est_deg * (M_PI / 180.0));
      }

      const double tau = std::max(1e-3, lpf_tau_sec_);
      const double alpha = dt / (tau + dt);

      if (!filt_inited_) {
        for (int i = 0; i < 4; ++i) theta_rad_filt_[i] = theta_rad_[i];
        filt_inited_ = true;
      } else {
        for (int i = 0; i < 4; ++i) {
          double target = theta_rad_[i];
          double prev   = theta_rad_filt_[i];
          double lpf_out = prev + alpha * (target - prev);

          double max_step = max_delta_rad_per_sec_ * dt;
          double delta = lpf_out - prev;
          if (delta >  max_step) lpf_out = prev + max_step;
          if (delta < -max_step) lpf_out = prev - max_step;

          theta_rad_filt_[i] = static_cast<float>(lpf_out);
        }
      }

      applyThetaToModel(theta_rad_filt_);

      // publish
      std_msgs::Float32MultiArray msg;
      msg.data.resize(4);
      for(int i = 0; i < 4; i++) msg.data[i] = theta_deg_[i];
      theta_pub_.publish(msg);

      ROS_INFO_STREAM_THROTTLE(0.5,
      "[ThetaEst] P=" << P << " kPa, T=["
      << latest_thrust_[0] << ", "
      << latest_thrust_[1] << ", "
      << latest_thrust_[2] << ", "
      << latest_thrust_[3] << "] N -> theta_deg=["
      << theta_rad_filt_[0] << ", "
      << theta_rad_filt_[1] << ", "
      << theta_rad_filt_[2] << ", "
      << theta_rad_filt_[3] << "]");


      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher  theta_pub_;
  ros::Publisher apply_to_joint_pub_, apply_to_joint_sim_pub_;
  ros::Subscriber pressure_sub_, thrust_sub_, mode_sub_;
  ThetaModel model_;

  float latest_pressure_{0.0f};
  std::vector<float> latest_thrust_;
  std::vector<float> theta_deg_;
  std::vector<float> theta_rad_;
  bool apply_estimate_mode_ = false;


  std::vector<float> theta_rad_filt_;
  bool filt_inited_ = false;
  ros::Time last_update_;
  double lpf_tau_sec_ = 0.20; //時定数
  double max_delta_rad_per_sec_ = 0.1;

  float latest_pressure_filt_;
  bool pressure_filt_inited_ = false;
  double pressure_lpf_sec_ = 1.00; //時定数
  double max_pressure_per_sec_ = 5;


  void pressureCb(const std_msgs::Float32::ConstPtr& msg){
    if (!msg) return;
    latest_pressure_ = msg->data;
  }
  // void thrustCb(const std_msgs::Float32::ConstPtr& msg){
  //   if (!msg) return;
  //   latest_thrust_ = msg->data;
  // }

  void thrustCb(const spinal::FourAxisCommand::ConstPtr& msg){
    if (!msg) return;
    const size_t n = std::min<size_t>(4, msg->base_thrust.size());
    for(int i = 0; i < n; i++){
      latest_thrust_[i] = std::abs(msg->base_thrust[i]) + 1.8;
    }
  }

  void modeCb(const std_msgs::Empty::ConstPtr& msg){
    if (msg && apply_estimate_mode_){
      apply_estimate_mode_ = false;
    }else if (msg && !apply_estimate_mode_){
      apply_estimate_mode_ = true;
    }else{
      return;
    }
  }


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
    model_.fit();
  }

  void applyThetaToModel(const std::vector<float>& theta_rad)
  {
    // KDL::JntArray q = robot_model->getJointPositions();
    // const auto& idx = robot_model->getJointIndexMap();

    sensor_msgs::JointState js, js_sim;
    js.header.stamp = ros::Time::now();
    js.name.reserve(12);
    js.position.reserve(12);

    js_sim.header.stamp = ros::Time::now();
    js_sim.name.reserve(12);
    js_sim.position.reserve(12);
 
    auto clamp = [](double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); };
    const double lim1 = 0.8727, limN = 1.047;

    const size_t arms = std::min<size_t>(4,theta_rad.size());
    for (int id = 1; id <= arms; ++id)
    {
      double theta = static_cast<double>(theta_rad.at(id-1));
      // double theta = static_cast<double>(theta_rad.at(0));
      double d = theta / 3.0;

      std::string j1 = "joint_" + std::to_string(id) + "_1";
      std::string j2 = "joint_" + std::to_string(id) + "_2";
      std::string j3 = "joint_" + std::to_string(id) + "_3";
      std::string j4 = "joint_" + std::to_string(id) + "_4";
      std::string j5 = "joint_" + std::to_string(id) + "_5";

      double v1,v2,v3;
      if (apply_estimate_mode_){
        v1 = clamp(d,0.0,lim1);
        v2 = clamp(d,0.0,limN);
        v3 = clamp(d,0.0,limN);
      }else{
        v1 = 0.0;
        v2 = 0.0;
        v3 = 0.0;
      }
      double v4 = 0.0;
      double v5 = 0.0;
      js.name.push_back(j1); js.position.push_back(v1);
      js.name.push_back(j2); js.position.push_back(v2);
      js.name.push_back(j3); js.position.push_back(v3);
      js.name.push_back(j4); js.position.push_back(v4);
      js.name.push_back(j5); js.position.push_back(v5);

      js_sim.name.push_back(j1); js_sim.position.push_back(v1);
      js_sim.name.push_back(j2); js_sim.position.push_back(v2);
      js_sim.name.push_back(j3); js_sim.position.push_back(v3);
      }
    // robot_model->updateRobotModel(q);
    apply_to_joint_pub_.publish(js);
    apply_to_joint_sim_pub_.publish(js_sim);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_model");
  JointModel node;
  node.spin();
  return 0;
}


