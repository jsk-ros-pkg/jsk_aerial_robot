//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_BI_2_ORD_SERVO_NMPC_CONTROLLER_H
#define TILT_BI_2_ORD_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_bi_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{
namespace nmpc_tilt_bi_2_ord
{

class NMPCController : public nmpc_tilt_bi_full::NMPCController
{
protected:
  std::vector<double> joint_vel_;

  void initCostW() override;

  inline void initJointStates() override
  {
    nmpc_over_act_full::NMPCController::initJointStates();
    joint_vel_.resize(joint_num_);
    for (int i = 0; i < joint_num_; i++)
      joint_vel_[i] = 0.0;
  }

  std::vector<double> meas2VecX() override;

  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;

  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::TiltBi2OrdServoMdlMPCSolver>();
  }

  void calXrUrRef(tf::Vector3 target_pos, tf::Vector3 target_vel, tf::Vector3 target_rpy, tf::Vector3 target_omega,
                  const Eigen::VectorXd& target_wrench) override;
};

};  // namespace nmpc_tilt_bi_2_ord

};  // namespace aerial_robot_control

#endif  // TILT_BI_2_ORD_SERVO_NMPC_CONTROLLER_H
