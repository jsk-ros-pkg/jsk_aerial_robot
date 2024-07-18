//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller_ekf.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwEKF::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  TiltQdServoNMPCwITerm::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);
}

void nmpc::TiltQdServoNMPCwEKF::initParams()
{
  TiltQdServoNMPC::initParams();

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
