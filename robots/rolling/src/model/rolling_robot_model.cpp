#include <rolling/model/rolling_robot_model.h>

RollingRobotModel::RollingRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  HydrusXiFullyActuatedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  rotors_coord_rotation_from_cog_.resize(rotor_num);
  links_rotation_from_cog_.resize(rotor_num);
  gimbal_nominal_angles_.resize(rotor_num);
}

void RollingRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);

  KDL::TreeFkSolverPos_recursive fk_solver(getTree());

  /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();

  /* link based on COG */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s);
      links_rotation_from_cog_[i] = cog_frame.Inverse() * f.M;
    }

  /* rotor's coordinate */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("rotor_coord") + s);
      rotors_coord_rotation_from_cog_[i] = cog_frame.Inverse() * f.M;
    }

  /* gimbal nominal angle */
  for(int i = 0; i < getRotorNum(); i++)
    {
      double r, p, y;
      links_rotation_from_cog_.at(i).GetRPY(r, p, y);
      gimbal_nominal_angles_.at(i) = -r;
    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RollingRobotModel, aerial_robot_model::RobotModel);
