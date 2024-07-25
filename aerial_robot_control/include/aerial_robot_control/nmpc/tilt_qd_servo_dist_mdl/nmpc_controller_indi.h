//
// Created by lijinjie on 24/07/25.
//

#ifndef TILT_QD_SERVO_NMPC_INDI_CONTROLLER_H
#define TILT_QD_SERVO_NMPC_INDI_CONTROLLER_H

#include "nmpc_controller.h"

#include "geometry_msgs/WrenchStamped.h"
#include "spinal/Imu.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoNMPCwINDI : public nmpc::TiltQdServoDistNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  ros::Subscriber sub_imu_;

  void calcDisturbWrench() override;

  void callbackImu(const spinal::ImuConstPtr& msg);
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_NMPC_INDI_CONTROLLER_H