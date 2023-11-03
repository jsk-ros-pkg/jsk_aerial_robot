// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/27.
//

#pragma once

#include "base/base.h"
#include "base/mpc_solver.h"
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

/* action */
#include "actionlib/server/simple_action_server.h"
#include "aerial_robot_msgs/PredXU.h"
#include "aerial_robot_msgs/TrackTrajAction.h"
#include "aerial_robot_msgs/TrackTrajFeedback.h"
#include "aerial_robot_msgs/TrackTrajGoal.h"
#include "aerial_robot_msgs/TrackTrajResult.h"

namespace aerial_robot_control
{

class MPCController : public ControlBase
{
public:
  MPCController();  // note that constructor should not have arguments as the rule of rospluginlib
  ~MPCController() override = default;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  bool update() override;
  void reset() override;
  void callback_viz_pred(const ros::TimerEvent& event);

protected:
  ros::Publisher pub_flight_cmd_;  // for spinal
  ros::Publisher pub_viz_pred_;    // for viz predictions
  ros::Timer tmr_viz_pred_;        // for viz predictions

private:
  double mass_;
  double gravity_const_;

  nav_msgs::Odometry odom_;
  aerial_robot_msgs::PredXU x_u_ref_;
  spinal::FourAxisCommand flight_cmd_;

  MPC::MPCSolver mpc_solver_;

  nav_msgs::Odometry getOdom();
  void sendFlightCmd();
};

};  // namespace aerial_robot_control
