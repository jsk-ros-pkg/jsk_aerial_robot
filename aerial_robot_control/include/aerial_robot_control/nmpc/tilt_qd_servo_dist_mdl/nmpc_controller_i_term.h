//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_QD_SERVO_NMPC_W_ITERM_CONTROLLER_H
#define TILT_QD_SERVO_NMPC_W_ITERM_CONTROLLER_H

#include "nmpc_controller.h"
#include "nmpc_solver.h"
#include "aerial_robot_control/nmpc/i_term.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoNMPCwITerm : public nmpc::TiltQdServoDistNMPC
{
protected:
  ITerm pos_i_term_[6];  // for x, y, z, roll, pitch, yaw

  void initParams() override;

  void calcDisturbWrench() override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_NMPC_W_ITERM_CONTROLLER_H