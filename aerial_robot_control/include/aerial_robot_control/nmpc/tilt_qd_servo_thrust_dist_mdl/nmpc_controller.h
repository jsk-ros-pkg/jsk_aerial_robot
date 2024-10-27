//
// Created by jinjie on 24/07/31.
//

#ifndef TILT_QD_SERVO_THRUST_DIST_NMPC_CONTROLLER_H
#define TILT_QD_SERVO_THRUST_DIST_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoThrustDistNMPC : public nmpc::TiltQdServoDistNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  double krpm2_d_thrust_;
  std::vector<double> thrust_meas_;
  ros::Subscriber sub_esc_telem_;

  inline void initActuatorStates() override
  {
    nmpc::TiltQdServoNMPC::initActuatorStates();
    thrust_meas_.resize(motor_num_, 0.0);
  }

  void initParams() override;

  void initCostW() override;

  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg);

  std::vector<double> meas2VecX() override;

  void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                    const tf::Vector3& ref_omega_b, const VectorXd& ref_wrench_b, vector<double>& x,
                    vector<double>& u) const override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_THRUST_DIST_NMPC_CONTROLLER_H
