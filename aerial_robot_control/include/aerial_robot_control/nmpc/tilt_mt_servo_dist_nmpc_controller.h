//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"
#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"
#include "aerial_robot_control/wrench_est/wrench_est_i_term.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

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

  bool update() override;

  aerial_robot_control::WrenchEstITerm wrench_est_i_term_;  // I term is indispensable to eliminate steady error.

  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::WrenchEstActuatorMeasBase>> wrench_est_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::WrenchEstActuatorMeasBase> wrench_est_ptr_;

protected:
  bool if_use_est_wrench_4_control_ = false;

  ros::Publisher pub_disturb_wrench_;  // for disturbance wrench

  int idx_p_dist_end_ = 0;

  void initPlugins() override;
  void resetPlugins() override;

  void initNMPCParams() override;

  void prepareNMPCParams() override;

  std::vector<double> meas2VecX(bool is_ee_centric) override;

  void initAllocMat() override;

  /* external wrench estimation */
  void updateDisturbWrench() const;

  void pubDisturbWrench() const;

  /* I Term */
  void updateITerm();
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_DIST_NMPC_CONTROLLER_H
