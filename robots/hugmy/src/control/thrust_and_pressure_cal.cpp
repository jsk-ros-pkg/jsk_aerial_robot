#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class ThetaModel{
public:
  static constexpr int POLY_DEGREE = 2;
  std::vector<double> P_grid;
  std::vector<double> T_grid;
  std::vector<std::vector<double>> theta_grid;
  Eigen::VectorXd coeffs;

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
};

class ThrustPressureCal{
public:
  ThrustPressureCal() : nh_("~")
  {
    theta_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/arm/theta", 1);
    thrust_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/arm/thrust_ref", 1);
    thrust_cur_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/quadrotor/arm/debug/target_thrust", 1, &ThrustPressureCal::targetThrustCallback, this);

    nh_.param("theta_target", theta_target_, 35.0);

    initModel();
  }

  void spin(){
    ros::Rate rate(1.0);
    while(ros::ok()){
      nh_.getParam("theta_target", theta_target_);
      auto [P_opt, T_opt] = model_.newton_find_PT_for_target(theta_target_, 0.0, thrust_cur_avr_ + 1.0);
      std::cout << "Thrust_current = " << thrust_cur_avr_ << "\n";
      int P_int = static_cast<int> (std::round(P_opt));
      std::cout << "Result: P= " << P_int << "kPa, T = " << T_opt
                <<"N, theta" << model_.f_theta(P_opt, T_opt) << "deg\n";
      double theta_est = model_.f_theta(P_opt, T_opt);

      std_msgs::Float32 msg_theta, msg_thrust;
      msg_theta.data = theta_est;
      msg_thrust.data = T_opt;
      theta_pub_.publish(msg_theta);
      thrust_pub_.publish(msg_thrust);

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher theta_pub_, thrust_pub_;
  ros::Subscriber thrust_cur_sub_;
  ThetaModel model_;

  double theta_target_;
  float thrust_cur_avr_ = 0.0;

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

  void targetThrustCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if (!msg || msg -> data.empty()) return;
    std::vector<float> thrust_cur(msg -> data.size());
    for(size_t i=0; i < thrust_cur.size(); ++i){
      thrust_cur[i] = msg -> data[i];
      thrust_cur_avr_ += thrust_cur[i];
    }
    thrust_cur_avr_ /= static_cast<float>(thrust_cur.size());
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "thrust_pressure_cal");
  ThrustPressureCal node;
  node.spin();
  return 0;
}
