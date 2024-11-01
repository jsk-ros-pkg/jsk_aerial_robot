//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_solver.h"
#include "aerial_robot_control/wrench_est/wrench_est_base.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltMtServoDistNMPC : public nmpc::TiltMtServoNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::WrenchEstBase>> wrench_est_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::WrenchEstBase> wrench_est_ptr_;

protected:
  ros::Publisher pub_disturb_wrench_;  // for disturbance wrench
  geometry_msgs::Vector3 dist_force_w_ = geometry_msgs::Vector3();
  geometry_msgs::Vector3 dist_torque_cog_ = geometry_msgs::Vector3();

  void initPlugins() override;

  std::vector<double> meas2VecX() override;

  virtual void calcDisturbWrench();

  void callbackViz(const ros::TimerEvent& event) override;

  void initAllocMat() override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H