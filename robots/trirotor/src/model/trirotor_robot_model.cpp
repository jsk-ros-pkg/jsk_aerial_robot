#include <trirotor/model/trirotor_robot_model.h>

TrirotorRobotModel::TrirotorRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
}

void TrirotorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  const auto joint_index_map = getJointIndexMap();
  gimbal_processed_joint_ = joint_positions;
  /* link based on COG */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      KDL::Frame f;
      links_rotation_from_cog_[i] = cog_frame.Inverse() * f.M;
    }
  /* normal robot model update */
  RobotModel::updateRobotModelImpl(gimbal_processed_joint_);

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrirotorRobotModel, aerial_robot_model::RobotModel);
