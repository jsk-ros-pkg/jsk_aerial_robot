//
// Created by lijinjie on 24/07/18.
//

#ifndef TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H
#define TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H

#include "nmpc_controller_i_term.h"
#include "aerial_robot_control/nmpc/kalman_filter.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoNMPCwEKF : public nmpc::TiltQdServoNMPCwITerm
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  void initParams() override;

  void calcDisturbWrench() override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H