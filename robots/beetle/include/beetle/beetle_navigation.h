// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>

namespace aerial_robot_navigation
{
  class BeetleNavigator : public GimbalrotorNavigator
  {
  public:
    BeetleNavigator();
    ~BeetleNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

    void update() override;

  };
};
