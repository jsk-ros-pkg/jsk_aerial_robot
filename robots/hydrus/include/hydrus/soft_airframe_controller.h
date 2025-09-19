#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>

namespace aerial_robot_control
{
class SoftAirframeController : public aerial_robot_control::PoseLinearController
{
public:
  SoftAirframeController();
  virtual ~SoftAirframeController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate);

  void controlCore() override;
  void sendCmd() override;
};
}  // namespace aerial_robot_control
