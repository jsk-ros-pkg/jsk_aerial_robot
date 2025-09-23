#include <hydrus/soft_airframe_controller.h>

using namespace aerial_robot_control;

SoftAirframeController::SoftAirframeController() : PoseLinearController()
{
}

void SoftAirframeController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                        boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                        double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
}

void SoftAirframeController::controlCore()
{
  PoseLinearController::controlCore();

  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftAirframeController] controlCore");
}

void SoftAirframeController::sendCmd()
{
  PoseLinearController::sendCmd();

  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftAirframeController] sendCmd");
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::SoftAirframeController, aerial_robot_control::ControlBase);
