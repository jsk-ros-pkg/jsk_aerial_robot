#include <ninja/model/ninja_robot_model.h>

NinjaRobotModel::NinjaRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  BeetleRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NinjaRobotModel, aerial_robot_model::RobotModel);
