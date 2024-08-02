//
// Created by jinjie on 24/08/01.
//

#ifndef TILT_QD_SERVO_THRUST_NMPC_INDI_CONTROLLER_INDI_H
#define TILT_QD_SERVO_THRUST_NMPC_INDI_CONTROLLER_INDI_H

#include "nmpc_controller.h"
#include "aerial_robot_control/nmpc/iir_filter.h"
#include "spinal/Imu.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoThrustNMPCwINDI : public nmpc::TiltQdServoThrustDistNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  double ts_servo_, ts_rotor_;
  ros::Subscriber sub_imu_;

  std::vector<IIRFilter> imu_acc_filters_;

  void initParams() override;

  void calcDisturbWrench() override;

  void sendCmd() override {}; // use INDI as the lower layer to send command

  void callbackImu(const spinal::ImuConstPtr& msg);
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_THRUST_NMPC_INDI_CONTROLLER_INDI_H
