#pragma once

#include <ros/ros.h>
#include <nlopt.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <numeric>
#include <vector>

class OptimalDesign
{
public:
  OptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~OptimalDesign() {}

  std::vector<double> theta_;
  int rotor_num_; // rotor number per unit
  double mass_; // mass per unit
  double m_f_rate_;
  double max_thrust_;
  double fc_f_min_weight_, fc_t_min_weight_;

  ros::Publisher visualize_pub_;
  ros::Publisher feasible_control_info_pub_;
  ros::Publisher feasible_control_info_pub_unit_;

  void setTheta(std::vector<double> theta) {theta_ = theta;}
};

double objectiveDesignFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
double objectiveModelingFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
double fxfyTConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
double fzConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr);

