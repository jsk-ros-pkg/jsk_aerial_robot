#include <rolling/model/rolling_robot_model.h>

RollingRobotModel::RollingRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  HydrusXiFullyActuatedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  rotors_coord_rotation_from_cog_.resize(rotor_num);
  links_rotation_from_cog_.resize(rotor_num);
  gimbal_nominal_angles_.resize(rotor_num);
  rotors_origin_from_contact_point_.resize(rotor_num);
  rotors_normal_from_contact_point_.resize(rotor_num);
  thrust_link_ = "thrust";
}

void RollingRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);

  const auto seg_tf_map = fullForwardKinematics(joint_positions);

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

  /* contact point */
  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  std::map<std::string, KDL::RigidBodyInertia> inertia_map = RobotModel::getInertiaMap();
  for(const auto& inertia : inertia_map)
    {
      KDL::Frame f = seg_tf_map.at(inertia.first);
      link_inertia = link_inertia + f * inertia.second;
    }

  KDL::Frame cog = getCog<KDL::Frame>();
  KDL::Frame cog2baselink = getCog2Baselink<KDL::Frame>();
  KDL::Vector contact_point_offset = KDL::Vector(0, -circle_radius_, 0);
  contact_point_.p = cog.p + cog.M * cog2baselink.M * contact_point_offset;
  contact_point_.M = cog.M;

  link_inertia_contact_point_ = (contact_point_.Inverse() * link_inertia).getRotationalInertia();

  for(int i = 0; i < getRotorNum(); i++)
    {
      std::string rotor = thrust_link_ + std::to_string(i + 1);
      KDL::Frame f = seg_tf_map.at(rotor);
      rotors_origin_from_contact_point_.at(i) = (contact_point_.Inverse() * f).p;
      // std::cout << rotors_origin_from_contact_point_.at(i).x() << " " << rotors_origin_from_contact_point_.at(i).y() << " " << rotors_origin_from_contact_point_.at(i).z() << std::endl;
      rotors_normal_from_contact_point_.at(i) = (contact_point_.Inverse() * f).M * KDL::Vector(0, 0, 1);
    }
  // std::cout << std::endl;

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RollingRobotModel, aerial_robot_model::RobotModel);
