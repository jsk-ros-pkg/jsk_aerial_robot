#include <crobat/model/crobat_robot_model.h>

CrobatRobotModel::CrobatRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CrobatRobotModel, aerial_robot_model::RobotModel);
