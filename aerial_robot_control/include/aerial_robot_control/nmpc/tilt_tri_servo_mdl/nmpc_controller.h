//
// Created by lijinjie on 24/06/03.
//

#ifndef TILT_TRI_SERVO_NMPC_CONTROLLER_H
#define TILT_TRI_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc_tilt_tri_full
{

class NMPCController : public nmpc_over_act_full::NMPCController
{
protected:
  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<nmpc::TiltTriServoMdlMPCSolver>();
  }

  void initAllocMat() override;

  void calXrUrRef(const tf::Vector3 target_pos, const tf::Vector3 target_vel, const tf::Vector3 target_rpy,
                  const tf::Vector3 target_omega, const Eigen::VectorXd& target_wrench) override;
};

};  // namespace nmpc_tilt_tri_full

};  // namespace aerial_robot_control

#endif  // TILT_TRI_SERVO_NMPC_CONTROLLER_H