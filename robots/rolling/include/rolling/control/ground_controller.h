// -*- mode: c++ -*-

#pragma once

namespace aerial_robot_control
{
  class GroundController : public PoseLinearController
  {
  public:
    GroundController();
    ~GroundController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;
  private:
    boost::shared_ptr<aerial_robot_navigation::RollingNavigator> rolling_navigator_;
    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;

  };
}
