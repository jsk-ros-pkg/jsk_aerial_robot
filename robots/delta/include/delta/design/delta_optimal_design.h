#pragma once

#include <ros/ros.h>
#include <nlopt.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class DeltaOptimalDesign
{
public:
  DeltaOptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~DeltaOptimalDesign(){}

  void run();
  std::vector<Eigen::Vector3d> calcRotorConfiguration(const std::vector<double>& theta);
  int rotor_num_; // rotor number per unit
  int eval_cnt_;
  double mass_;
  double max_thrust_;
  double theta_max_;
  double m_f_rate_;
  double fct_min_weight_;
  double gimbal_penalty_weight_;
  std::vector<double> theta_;
  std::vector<double> direction_;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  template<class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }

};

double objectiveDesignFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
// double objectiveModelingFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
