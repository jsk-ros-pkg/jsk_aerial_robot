// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/27.
//

#pragma once

#include "aerial_robot_control/control/base/base.h"
#include "aerial_robot_control/nmpc/unactuated_body_rate//mpc_solver.h"
// #include <aerial_robot_control/control/utils/pid.h>
// #include <aerial_robot_control/PIDConfig.h>

#include "angles/angles.h"
#include "dynamic_reconfigure/server.h"
// using PidControlDynamicConfig =
// dynamic_reconfigure::Server<aerial_robot_control::PIDConfig>;

/* protocol */
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "aerial_robot_msgs/PredXU.h"
#include "spinal/FourAxisCommand.h"
#include "spinal/RollPitchYawTerms.h"
#include "spinal/PMatrixPseudoInverseWithInertia.h"
#include "spinal/SetControlMode.h"

/* action */
#include "actionlib/server/simple_action_server.h"
#include "aerial_robot_msgs/PredXU.h"
#include "aerial_robot_msgs/TrackTrajAction.h"
#include "aerial_robot_msgs/TrackTrajFeedback.h"
#include "aerial_robot_msgs/TrackTrajGoal.h"
#include "aerial_robot_msgs/TrackTrajResult.h"

namespace aerial_robot_control
{

class NMPCController : public ControlBase
{
public:
  NMPCController();  // note that constructor should not have arguments as the rule of rospluginlib
  ~NMPCController() override = default;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  bool update() override;
  void reset() override;
  void callbackViz(const ros::TimerEvent& event);

  void sendRPYGain();
  void sendRotationalInertiaComp();

protected:
  ros::Timer tmr_viz_;

  ros::Publisher pub_flight_cmd_;                       // for spinal
  ros::Publisher pub_viz_pred_;                         // for viz predictions
  ros::Publisher pub_viz_ref_;                          // for viz reference
  ros::Publisher pub_rpy_gain_;                         // for gains of attitude controller
  ros::Publisher pub_p_matrix_pseudo_inverse_inertia_;  // for pseudo inverse inertia

  ros::ServiceClient srv_set_control_mode_;

  virtual void controlCore();
  virtual void sendCmd();

private:
  double mass_;
  double gravity_const_;
  double t_nmpc_samp_;
  double t_nmpc_integ_;

  double yaw_p_gain, yaw_d_gain;  // TODO: no need to use these parameters in the future

  double target_roll_, target_pitch_;
  double candidate_yaw_term_;

  nav_msgs::Odometry odom_;
  aerial_robot_msgs::PredXU x_u_ref_;
  spinal::FourAxisCommand flight_cmd_;

  MPC::MPCSolver mpc_solver_;

  nav_msgs::Odometry getOdom();
};

};  // namespace aerial_robot_control
