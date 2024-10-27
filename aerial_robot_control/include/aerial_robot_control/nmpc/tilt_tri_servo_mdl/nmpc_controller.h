//
// Created by lijinjie on 24/06/03.
//

#ifndef TILT_TRI_SERVO_NMPC_CONTROLLER_H
#define TILT_TRI_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltTriServoNMPC : public nmpc::TiltQdServoNMPC
{
protected:
  void initAllocMat() override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_TRI_SERVO_NMPC_CONTROLLER_H