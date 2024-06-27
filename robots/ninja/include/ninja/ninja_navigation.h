// -*- mode: c++ -*-

#pragma once

#include <beetle/beetle_navigation.h>

namespace aerial_robot_navigation
{
  class NinjaNavigator : public BeetleNavigator
  {
  public:
    NinjaNavigator();
    ~NinjaNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;
  private:
    void convertTargetPosFromCoG2CoM() override;
    ros::Subscriber entire_structure_sub_;
    ros::Publisher joint_control_pub_;

  };
};
