#include <beetle/model/beetle_robot_model.h>

BeetleRobotModel::BeetleRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

void BeetleRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);

  const auto seg_tf_map = getSegmentsTf();
  KDL::Frame fix_cp_frame = seg_tf_map.at("contact_point");
  KDL::Frame variable_cp_frame;
  variable_cp_frame.M = fix_cp_frame.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  variable_cp_frame.p = fix_cp_frame.p;
  setContactFrame(variable_cp_frame);
  KDL::Frame Cog2Cp;
  setCog2Cp(getCog<KDL::Frame>().Inverse() * variable_cp_frame);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BeetleRobotModel, aerial_robot_model::RobotModel);
