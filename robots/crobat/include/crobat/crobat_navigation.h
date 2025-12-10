// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>

namespace aerial_robot_navigation
{
  class CrobatNavigator : public GimbalrotorNavigator
  {
  public:
    CrobatNavigator();
    ~CrobatNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;
  };
};
