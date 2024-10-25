// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>
#include "beetle/model/beetle_omni_robot_model.h"
#include <diagnostic_msgs/KeyValue.h>

namespace aerial_robot_navigation
{

class BeetleOmniNavigator : public BaseNavigator
{
public:
  BeetleOmniNavigator() : BaseNavigator() {};
  ~BeetleOmniNavigator() override = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::shared_ptr<BeetleOmniRobotModel> beetle_robot_model_;
};
};  // namespace aerial_robot_navigation