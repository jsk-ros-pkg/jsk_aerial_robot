//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_PLUGINS_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_PLUGINS_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include <cmath>

namespace aerial_robot_control
{

class WrenchEstITerm : public WrenchEstBase
{
public:
  WrenchEstITerm() = default;

  void initialize(ros::NodeHandle nh, double ctrl_loop_du) override
  {
    double du = getCtrlLoopDu();
    ROS_INFO("WrenchEstITerm initialize %f", du);
  }

  void update(geometry_msgs::Pose pose_ref, geometry_msgs::Pose pose) override
  {
    // do nothing
  }

private:
};

class WrenchEstAcc : public aerial_robot_control::WrenchEstBase
{
public:
  WrenchEstAcc() = default;

  void initialize(ros::NodeHandle nh, double ctrl_loop_du) override
  {
    ROS_INFO("WrenchEstAcc initialize");
  }

  void update(geometry_msgs::Pose pose_ref, geometry_msgs::Pose pose) override
  {
    // do nothing
  }

private:
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_PLUGINS_H
