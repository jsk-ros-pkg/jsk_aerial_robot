#pragma once

#include <pinocchio/fwd.hpp>  // should be included before any other pinocchio headers
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/force.hpp>

#include <aerial_robot_dynamics/math_utils.h>

#include <OsqpEigen/OsqpEigen.h>

#include <chrono>
#include <urdf/model.h>
#include <ros/ros.h>
#include <tinyxml.h>
#include <iostream>
#include <memory>

namespace aerial_robot_dynamics
{
class PinocchioRobotModel
{
public:
  PinocchioRobotModel(bool is_floating_base = true);
  ~PinocchioRobotModel() = default;

  std::shared_ptr<pinocchio::Model> getModel() const
  {
    return model_;
  }
  std::shared_ptr<pinocchio::Data> getData() const
  {
    return data_;
  }

  Eigen::VectorXd forwardDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& tau,
                                  Eigen::VectorXd& thrust);
  Eigen::MatrixXd forwardDynamicsDerivatives(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                             const Eigen::VectorXd& tau, Eigen::VectorXd& thrust);
  bool inverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                       Eigen::VectorXd& tau);
  bool inverseDynamicsDerivatives(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                                  Eigen::MatrixXd& id_partial_dq, Eigen::MatrixXd& id_partial_dv,
                                  Eigen::MatrixXd& id_partial_da);

  pinocchio::container::aligned_vector<pinocchio::Force>
  computeFExtByThrust(const Eigen::VectorXd& thrust);  // external force is expressed in the LOCAL frame
  std::vector<Eigen::MatrixXd> computeTauExtByThrustDerivativeQDerivatives(const Eigen::VectorXd& q);
  std::vector<Eigen::MatrixXd> computeTauExtByThrustDerivativeQDerivativesNum(const Eigen::VectorXd& q);
  Eigen::MatrixXd computeTauExtByThrustDerivative(const Eigen::VectorXd& q);

  const bool& getIsFloatingBase() const
  {
    return is_floating_base_;
  }
  const int& getRotorNum() const
  {
    return rotor_num_;
  }
  const double& getMFRate() const
  {
    return m_f_rate_;
  }
  const double& getLatestIdSolveTime() const
  {
    return latest_id_solve_time_;
  }
  const int& getRotorDirection(int index) const
  {
    return rotor_direction_.at(index);
  }
  const Eigen::VectorXd& getJointTorqueLimits() const
  {
    return joint_torque_limits_;
  }
  const Eigen::VectorXd& getThrustUpperLimits() const
  {
    return thrust_upper_limits_;
  }
  const Eigen::VectorXd& getThrustLowerLimits() const
  {
    return thrust_lower_limits_;
  }
  const double getThrustHessianWeight() const
  {
    return thrust_hessian_weight_;
  }
  Eigen::VectorXd getResetConfiguration();

private:
  urdf::Model urdf_;
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::vector<std::string> rotor_names_;
  std::vector<pinocchio::SE3> joint_M_rotors_;
  std::vector<int> rotor_direction_;

  // QP solver for Inverse Dynamics
  double latest_id_solve_time_ = 0.0;
  OsqpEigen::Solver id_solver_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

  // model parameters
  bool is_floating_base_;
  int rotor_num_;
  double m_f_rate_;
  Eigen::VectorXd joint_torque_limits_;
  Eigen::VectorXd thrust_upper_limits_;
  Eigen::VectorXd thrust_lower_limits_;

  // ID solver parameters
  double thrust_hessian_weight_;

  bool getRobotModelXml(const std::string& param_name, std::string& pinocchio_robot_description,
                        ros::NodeHandle nh = ros::NodeHandle());

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};
}  // namespace aerial_robot_dynamics
