#include <beetle_omni/model/beetle_omni_robot_model.h>

BeetleOmniRobotModel::BeetleOmniRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
}

void BeetleOmniRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  // /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  RobotModel::updateRobotModelImpl(joint_positions);
  const auto seg_tf_map = getSegmentsTf();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BeetleOmniRobotModel, aerial_robot_model::RobotModel);
