#include <rolling/control/ground_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

GroundController::GroundController():
  PoseLinearController()
{
}

void GroundController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rolling_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::RollingNavigator>(navigator_);
  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GroundController, aerial_robot_control::ControlBase);

