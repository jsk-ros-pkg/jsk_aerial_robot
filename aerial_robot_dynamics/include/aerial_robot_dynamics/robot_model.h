#pragma once

#include <pinocchio/fwd.hpp>  // should be included before any other pinocchio headers
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <OsqpEigen/OsqpEigen.h>

// osqp/constants.h defines WARM_START as a macro, which mangles
// proxsuite::proxqp::InitialGuessStatus::WARM_START and breaks the whole enum
#ifdef WARM_START
#undef WARM_START
#endif
#include <proxsuite/proxqp/dense/dense.hpp>

#include <chrono>
#include <urdf/model.h>
#include <tinyxml.h>
#include <iostream>
#include <memory>

namespace aerial_robot_dynamics
{
class PinocchioRobotModel
{
public:
  struct Config
  {
    double thrust_hessian_weight = 1.0;
    bool verbose = true;
    bool warm_start_inverse_dynamics = true;
  };

  PinocchioRobotModel(std::string robot_description, std::string pinocchio_robot_description, bool is_floating_base);
  PinocchioRobotModel(std::string robot_description, std::string pinocchio_robot_description, bool is_floating_base,
                      const Config& config);
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
  [[deprecated("Use inverseDynamicsOsqp instead.")]] bool
  inverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a, Eigen::VectorXd& tau)
  {
    return inverseDynamicsOsqp(q, v, a, tau);
  }
  bool inverseDynamicsOsqp(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                           Eigen::VectorXd& tau, const Eigen::VectorXd& base_residual_lower = Eigen::VectorXd::Zero(6),
                           const Eigen::VectorXd& base_residual_upper = Eigen::VectorXd::Zero(6));
  bool inverseDynamicsProxqp(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                             Eigen::VectorXd& tau,
                             const Eigen::VectorXd& base_residual_lower = Eigen::VectorXd::Zero(6),
                             const Eigen::VectorXd& base_residual_upper = Eigen::VectorXd::Zero(6));

  std::vector<pinocchio::Force> computeFExtByThrust(const Eigen::VectorXd& thrust);  // external force is expressed in
                                                                                     // the LOCAL frame
  std::vector<Eigen::MatrixXd> computeTauExtByThrustDerivativeQDerivatives(const Eigen::VectorXd& q);
  std::vector<Eigen::MatrixXd> computeTauExtByThrustDerivativeQDerivativesNum(const Eigen::VectorXd& q);
  Eigen::VectorXd computeTauExtByThrust(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust);
  Eigen::MatrixXd computeTauExtByThrustDerivative(const Eigen::VectorXd& q);
  [[deprecated("Use computeTauExtByThrustQDerivativeRnea instead.")]] Eigen::MatrixXd
  computeTauExtByThrustQDerivative(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust)
  {
    return computeTauExtByThrustQDerivativeRnea(q, thrust);
  }
  Eigen::MatrixXd computeTauExtByThrustQDerivativeRnea(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust);
  Eigen::MatrixXd computeTauExtByThrustQDerivativeStaticTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust);
  Eigen::MatrixXd computeTauExtByThrustQDerivativeHessian(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust);
  Eigen::MatrixXd computeTauExtByThrustQDerivativeNum(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust);

  const std::string& getRobotDescription() const
  {
    return robot_description_;
  }
  const std::string& getPinocchioRobotDescription() const
  {
    return pinocchio_robot_description_;
  }
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
  const std::vector<pinocchio::SE3>& getJointMRotors() const
  {
    return joint_M_rotors_;
  }
  const int& getRotorDirection(int index) const
  {
    return rotor_direction_.at(index);
  }
  const std::vector<int>& getRotorFrameIndices() const
  {
    return rotor_frame_indices_;
  }
  const std::vector<std::string>& getRotorNames() const
  {
    return rotor_names_;
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
  const Config& getConfig() const
  {
    return config_;
  }
  const double getThrustHessianWeight() const
  {
    return config_.thrust_hessian_weight;
  }
  Eigen::VectorXd getResetConfiguration();

private:
  std::string robot_description_;
  std::string pinocchio_robot_description_;
  urdf::Model urdf_;
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Model> zero_gravity_model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::vector<std::string> rotor_names_;
  std::vector<pinocchio::SE3> joint_M_rotors_;
  std::vector<int> rotor_direction_;
  std::vector<int> rotor_frame_indices_;

  // QP solver for Inverse Dynamics
  double latest_id_solve_time_ = 0.0;
  OsqpEigen::Solver id_solver_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

  // ProxQP solver for Inverse Dynamics
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> id_solver_proxqp_;

  // model parameters
  bool is_floating_base_ = true;
  int rotor_num_;
  double m_f_rate_ = 0.0;
  Eigen::VectorXd joint_torque_limits_;
  Eigen::VectorXd thrust_upper_limits_;
  Eigen::VectorXd thrust_lower_limits_;

  Config config_;

  // joint torque box for the QP
  void jointTorqueBounds(const Eigen::VectorXd& base_residual_lower, const Eigen::VectorXd& base_residual_upper,
                         Eigen::VectorXd& lower, Eigen::VectorXd& upper) const;
};
}  // namespace aerial_robot_dynamics
