#include <beetle/control/beetle_controller.h>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController()
  {
  }

  void BeetleController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                         boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                         double ctrl_loop_rate
                                         )
  {
    GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  }
} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
