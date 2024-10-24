//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"

namespace aerial_robot_control
{

class WrenchEstMomentum : public aerial_robot_control::WrenchEstBase
{
public:
  WrenchEstMomentum() = default;

  void initialize(ros::NodeHandle nh, double ctrl_loop_du) override
  {
    ROS_INFO("WrenchEstMomentum initialize");
  }

  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override
  {
    // do nothing
  }

private:
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
