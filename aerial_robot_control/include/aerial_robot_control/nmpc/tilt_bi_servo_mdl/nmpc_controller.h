//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_BI_SERVO_NMPC_CONTROLLER_H
#define TILT_BI_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltBiServoNMPC : public nmpc::TiltQdServoNMPC
{
protected:
  void initAllocMat() override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_BI_SERVO_NMPC_CONTROLLER_H
