//
// Created by li-jinjie on 24-11-21.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_imp_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoThrustImpNMPC::initCostW()
{
  TiltMtServoThrustDistNMPC::initCostW();
}

void nmpc::TiltMtServoThrustImpNMPC::prepareNMPCParams()
{
}

void nmpc::TiltMtServoThrustImpNMPC::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustImpNMPC, aerial_robot_control::ControlBase)