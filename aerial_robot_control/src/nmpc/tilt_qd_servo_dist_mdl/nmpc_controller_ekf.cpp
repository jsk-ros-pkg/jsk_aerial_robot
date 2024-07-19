//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller_ekf.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwEKF::initParams()
{
  TiltQdServoNMPC::initParams();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle ekf_nh(control_nh, "ekf");

  /* EKF */
}

void nmpc::TiltQdServoNMPCwEKF::calcDisturbWrench()
{
}

void nmpc::TiltQdServoNMPCwEKF::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  TiltQdServoNMPC::cfgNMPCCallback(config, level);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoNMPCwEKF, aerial_robot_control::ControlBase);
