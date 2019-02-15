#include <hydrus_xi/hydrus_xi_lqi_controller.h>

HydrusXiLqiController::HydrusXiLqiController(ros::NodeHandle nh, ros::NodeHandle nh_private, std::unique_ptr<HydrusXiRobotModel> robot_model):
  TransformController(nh, nh_private, std::move(robot_model))
{

}
