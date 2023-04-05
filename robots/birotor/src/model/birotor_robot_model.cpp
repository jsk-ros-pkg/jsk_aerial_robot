#include <birotor/model/birotor_robot_model.h>

BirotorRobotModel::BirotorRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BirotorRobotModel, aerial_robot_model::RobotModel);
