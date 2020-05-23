#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>


HydrusXiFullyActuatedRobotModel::HydrusXiFullyActuatedRobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HydrusXiFullyActuatedRobotModel, aerial_robot_model::RobotModel);
