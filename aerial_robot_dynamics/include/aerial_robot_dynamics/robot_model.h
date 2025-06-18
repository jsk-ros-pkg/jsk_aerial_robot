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

  std::vector<Eigen::MatrixXd> computeTauExtByThrustDerivativeQDerivatives(const Eigen::VectorXd& q);
  std::vector<Eigen::MatrixXd> computeTauExtByThrustDerivativeQDerivativesNum(const Eigen::VectorXd& q);

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
  Eigen::VectorXd getResetConfiguration();

private:
  urdf::Model urdf_;
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::SE3> joint_M_rotors_;

  // QP solver for Inverse Dynamics
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
  double thrust_hessian_weight_ = 0.1;

  Eigen::MatrixXd computeTauExtByThrustDerivative(const Eigen::VectorXd& q);
  pinocchio::container::aligned_vector<pinocchio::Force>
  computeFExtByThrust(const Eigen::VectorXd& thrust);  // external force is expressed in the LOCAL frame

  std::string getRobotModelXml(const std::string& param_name, ros::NodeHandle nh = ros::NodeHandle());
};
}  // namespace aerial_robot_dynamics
