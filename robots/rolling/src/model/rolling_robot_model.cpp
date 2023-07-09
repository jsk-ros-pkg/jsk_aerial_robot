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
  rotors_x_axis_from_cog_.resize(rotor_num);
  rotors_y_axis_from_cog_.resize(rotor_num);
  thrust_link_ = "thrust";

  ros::NodeHandle nh;
  feasible_control_force_pub_  = nh.advertise<geometry_msgs::PoseArray>("feasible_control_force_convex", 1);
  feasible_control_torque_pub_ = nh.advertise<geometry_msgs::PoseArray>("feasible_control_torque_convex", 1);
  feasible_control_force_radius_pub_  = nh.advertise<std_msgs::Float32>("feasible_control_force_radius", 1);
  feasible_control_torque_radius_pub_ = nh.advertise<std_msgs::Float32>("feasible_control_torque_radius", 1);
  rotor_origin_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug/rotor_origin", 1);
  rotor_normal_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug/rotor_normal", 1);
}

void RollingRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);

  const auto seg_tf_map = fullForwardKinematics(joint_positions);

  KDL::TreeFkSolverPos_recursive fk_solver(getTree());

  /* special process */
  KDL::Frame f_baselink;
  KDL::Frame cog = getCog<KDL::Frame>();
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

  // rotor's y and z axis
  for(int i = 0; i < getRotorNum(); i++)
    {
      std::string rotor = thrust_link_ + std::to_string(i + 1);
      KDL::Frame f = seg_tf_map.at(rotor);
      rotors_x_axis_from_cog_.at(i) = (cog.Inverse() * f).M * KDL::Vector(1, 0, 0);
      rotors_y_axis_from_cog_.at(i) = (cog.Inverse() * f).M * KDL::Vector(0, 1, 0);
    }

  /* rotor's neutral coordinate */
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
      rotors_normal_from_contact_point_.at(i) = (contact_point_.Inverse() * f).M * KDL::Vector(0, 0, 1);
    }


  geometry_msgs::PoseArray feasible_force_array_msg;
  geometry_msgs::PoseArray feasible_torque_array_msg;
  std_msgs::Float32 feasible_force_radius_msg;
  std_msgs::Float32 feasible_torque_radius_msg;

  std::vector<Eigen::Vector3d> v;
  auto t_min_list = calcFeasibleControlTDists(v);
  geometry_msgs::PoseArray fc_torque_array_msg;
  std_msgs::Float32 fc_torque_radius_msg;
  for(const auto& torque_v: v)
    {
      geometry_msgs::Pose tmp;
      tmp.position.x = torque_v.x();
      tmp.position.y = torque_v.y();
      tmp.position.z = torque_v.z();
      fc_torque_array_msg.poses.push_back(tmp);
    }
  fc_torque_radius_msg.data = t_min_list.minCoeff();

  feasible_control_torque_pub_.publish(fc_torque_array_msg);
  feasible_control_torque_radius_pub_.publish(fc_torque_radius_msg);

  feasible_control_force_pub_.publish(feasible_force_array_msg);
  feasible_control_force_radius_pub_.publish(feasible_force_radius_msg);
}

Eigen::VectorXd RollingRobotModel::calcFeasibleControlTDists(std::vector<Eigen::Vector3d>&v)
{
  v.clear();
  const int rotor_num = getRotorNum();
  const auto& sigma = getRotorDirection();
  const double m_f_rate = getMFRate();
  for(int i = 0; i < rotor_num; i++)
    {
      Eigen::Vector3d p = getRotorsOriginFromCog<Eigen::Vector3d>().at(i);
      Eigen::Vector3d y_axis_i = kdlToEigen(rotors_coord_rotation_from_cog_.at(i)).col(1);
      Eigen::Vector3d z_axis_i = kdlToEigen(rotors_coord_rotation_from_cog_.at(i)).col(2);
      v.push_back(p.cross(y_axis_i));
      v.push_back(p.cross(z_axis_i) + m_f_rate * sigma.at(i + 1) * z_axis_i);
    }

  // std::cout << "size of v = " << v.size() << std::endl;
  Eigen::VectorXd t_min(v.size() * (v.size() - 1) / 2);
  int t_min_index = 0;
  for(int i = 0; i < v.size(); ++i)
    {
      for(int j = i + 1; j < v.size(); ++j)
        {
          if(i == j) continue;
          double t_min_ij = 0.0;
          if(v.at(i).cross(v.at(j)).norm() < 1e-5)
            {
              t_min_ij = 1e6;
              ROS_DEBUG_NAMED("robot_model", "the direction of v%d and v%d are too close, so no plane can generate by these two vector", i , j);
            }
          else
            {
              for(int k = 0; k < v.size(); ++k)
                {
                  if(i == k || j == k) continue;
                  double v_triple_product = calcTripleProduct(v.at(i), v.at(j), v.at(k));
                  t_min_ij += fabs(v_triple_product);
                }
            }
          t_min(t_min_index) = t_min_ij;
          t_min_index++;
        }
    }
  return t_min;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RollingRobotModel, aerial_robot_model::RobotModel);
