//
// Created by li-jinjie on 24-9-28.
//

#ifndef TILT_QD_SERVO_THRUST_NMPC_W_ITERM_CONTROLLER_H
#define TILT_QD_SERVO_THRUST_NMPC_W_ITERM_CONTROLLER_H

#include "nmpc_controller.h"
#include "aerial_robot_control/wrench_est/i_term.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoThrustNMPCwITerm : public TiltQdServoThrustDistNMPC
{
protected:
  ITerm pos_i_term_[6];  // for x, y, z, roll, pitch, yaw

  void initParams() override;

  void calcDisturbWrench() override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_THRUST_NMPC_W_ITERM_CONTROLLER_H
