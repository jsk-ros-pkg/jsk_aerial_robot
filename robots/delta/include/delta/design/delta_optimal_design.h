#pragma once

#include <ros/ros.h>
#include <nlopt.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <delta/model/delta_robot_model.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

class DeltaOptimalDesign
{
public:
  DeltaOptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~DeltaOptimalDesign(){}

  void run();
  std::vector<Eigen::Vector3d> calcRotorConfiguration(const std::vector<double>& theta);
  boost::shared_ptr<RollingRobotModel> delta_robot_model_;
  int eval_cnt_;
  double theta_max_;
  double fct_min_weight_;
  double gimbal_penalty_weight_;
  std::vector<double> theta_;
  std::map<int, int> rotor_direction_;
  std::vector<Eigen::Vector3d> rotors_origin_from_cog_;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher feasible_control_torque_convex_pub_;
  ros::Publisher feasible_control_torque_radius_pub_;

  template<class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }

};

double objectiveDesignFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
// double objectiveModelingFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr);
