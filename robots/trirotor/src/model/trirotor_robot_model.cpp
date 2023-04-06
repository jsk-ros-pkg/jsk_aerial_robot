#include <trirotor/model/trirotor_robot_model.h>

TrirotorRobotModel::TrirotorRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrirotorRobotModel, aerial_robot_model::RobotModel);
