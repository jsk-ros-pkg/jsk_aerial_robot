//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_BI_SERVO_NMPC_CONTROLLER_H
#define TILT_BI_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc_tilt_bi_full
{

class NMPCController : public nmpc::TiltQdServoNMPC
{
protected:
  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::TiltBiServoMdlMPCSolver>();
  }

  void initAllocMat() override;

  void calXrUrRef(tf::Vector3 target_pos, tf::Vector3 target_vel, tf::Vector3 target_rpy, tf::Vector3 target_omega,
                  const Eigen::VectorXd& target_wrench) override;
};

}  // namespace nmpc_tilt_bi_full

}  // namespace aerial_robot_control

#endif  // TILT_BI_SERVO_NMPC_CONTROLLER_H
