//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_BI_2_ORD_SERVO_NMPC_CONTROLLER_H
#define TILT_BI_2_ORD_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_bi_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{
namespace nmpc
{

class TiltBi2OrdServoNMPC : public nmpc::TiltBiServoNMPC
{
protected:
  std::vector<double> joint_vel_;

  void initCostW() override;

  inline void initActuatorStates() override
  {
    nmpc::TiltQdServoNMPC::initActuatorStates();
    joint_vel_.resize(joint_num_, 0);
  }

  std::vector<double> meas2VecX() override;

  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

};  // namespace nmpc

};  // namespace aerial_robot_control

#endif  // TILT_BI_2_ORD_SERVO_NMPC_CONTROLLER_H
