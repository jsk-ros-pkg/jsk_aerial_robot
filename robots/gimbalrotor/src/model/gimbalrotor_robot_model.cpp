#include <gimbalrotor/model/gimbalrotor_robot_model.h>

GimbalrotorRobotModel::GimbalrotorRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
  thrust_coords_rot_.resize(rotor_num);
}

void GimbalrotorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  // /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  transformable::RobotModel::updateRobotModelImpl(joint_positions);
  const auto seg_tf_map = getSegmentsTf();

  /* get local coords of thrust links */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string thrust = "rotor_arm" + std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, thrust);
      thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;

    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GimbalrotorRobotModel, aerial_robot_model::RobotModel);
