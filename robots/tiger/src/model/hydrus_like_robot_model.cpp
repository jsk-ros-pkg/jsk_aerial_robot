#include <tiger/model/hydrus_like_robot_model.h>

using namespace Tiger;

HydrusLikeRobotModel::HydrusLikeRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double fc_rp_min_thre, double epsilon, double edf_radius, double edf_max_tilt) : Dragon::HydrusLikeRobotModel(init_with_rosparam, verbose, fc_t_min_thre, fc_rp_min_thre, epsilon, edf_radius, edf_max_tilt)
{
}


void HydrusLikeRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  Dragon::HydrusLikeRobotModel::updateRobotModelImpl(joint_positions);

  calcJointTorque(true);

  const auto& joint_torque = getJointTorque();
  ROS_INFO_THROTTLE(1.0, "joint static torque for Limb1 is: [%f, %f, %f, %f]", joint_torque(0), joint_torque(1), joint_torque(4), joint_torque(5));
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Tiger::HydrusLikeRobotModel, aerial_robot_model::RobotModel);
