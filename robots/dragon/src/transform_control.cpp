#include <dragon/transform_control.h>

DragonTransformController::DragonTransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, std::unique_ptr<DragonRobotModel> robot_model):
  TransformController(nh, nh_private, std::move(robot_model))
{

}
