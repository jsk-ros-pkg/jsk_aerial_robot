#include <beetle_omni/model/beetle_omni_robot_model.h>

BeetleOmniRobotModel::BeetleOmniRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

void BeetleOmniRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);

  if (hasEEContact())
  {
    updateCoGtoEEContact();
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BeetleOmniRobotModel, aerial_robot_model::RobotModel);
