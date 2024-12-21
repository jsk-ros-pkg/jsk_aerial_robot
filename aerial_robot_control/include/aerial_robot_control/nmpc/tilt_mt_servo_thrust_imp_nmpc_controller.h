//
// Created by li-jinjie on 24-11-21.
//

#ifndef TILT_MT_SERVO_THRUST_IMP_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_THRUST_IMP_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_dist_nmpc_controller.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_imp_mdl/nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltMtServoThrustImpNMPC : public TiltMtServoThrustDistNMPC
{
protected:
  void prepareNMPCParams() override;

  std::vector<double> meas2VecX() override;

  void initCostW() override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_THRUST_IMP_NMPC_CONTROLLER_H
