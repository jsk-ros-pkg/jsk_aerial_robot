//
// Created by li-jinjie on 23-11-25.
//

#ifndef Fix_QD_NMPC_CONTROLLER_H
#define Fix_QD_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc
{

class FixQdNMPC : public nmpc::TiltQdServoNMPC
{
protected:
  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::FixQdMdlMPCSolver>();
  }

  void initAllocMat() override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // Fix_QD_NMPC_CONTROLLER_H
