#include <rolling/model/rolling_robot_model.h>

RollingRobotModel::RollingRobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  // rotors_coord_rotation_from_cog_.resize(rotor_num);
  links_rotation_from_cog_.resize(rotor_num);
  links_rotation_from_target_frame_.resize(rotor_num);
  // gimbal_nominal_angles_.resize(rotor_num);
  rotors_origin_from_target_frame_.resize(rotor_num);
  rotors_normal_from_target_frame_.resize(rotor_num);
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

  additional_frame_["cp"] = &contact_point_;
}

void RollingRobotModel::calcRobotModelFromFrame(std::string frame_name)
{
  /* get target frame from name */
  target_frame_name_ = frame_name;
  if(frame_name == "cog")
    {
      links_rotation_from_target_frame_ = links_rotation_from_cog_;
      link_inertia_from_target_frame_ = getInertia<KDL::RotationalInertia>();
      rotors_origin_from_target_frame_ = getRotorsOriginFromCog<KDL::Vector>();
      return;
    }

  const std::map<std::string, KDL::Frame> seg_tf_map = getSegmentsTf();
  if(seg_tf_map.find(frame_name) == seg_tf_map.end() && additional_frame_.find(frame_name) == additional_frame_.end())
    {
      ROS_ERROR_STREAM("[model] there is not frame named " << frame_name);
      return;
    }
  KDL::Frame target_frame;
  if(seg_tf_map.find(frame_name) == seg_tf_map.end()) target_frame = *(additional_frame_.at(frame_name));
  else target_frame = seg_tf_map.at(frame_name);

  /* get cog */
  KDL::Frame cog = getCog<KDL::Frame>();

  /* calculate link rotation from target frame */
  for(int i = 0;  i < getRotorNum(); i++)
    {
      links_rotation_from_target_frame_.at(i) = target_frame.M.Inverse() *  cog.M * links_rotation_from_cog_.at(i);
    }

  /* calculate inertia from target frame */
  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  std::map<std::string, KDL::RigidBodyInertia> inertia_map = RobotModel::getInertiaMap();
  for(const auto& inertia : inertia_map)
    {
      KDL::Frame f = seg_tf_map.at(inertia.first);
      link_inertia = link_inertia + f * inertia.second;
    }
  link_inertia_from_target_frame_ = (target_frame.Inverse() * link_inertia).getRotationalInertia();

  /* origin and normal of rotor */
  for(int i = 0 ; i < getRotorNum(); i++)
    {
      std::string rotor = thrust_link_ + std::to_string(i + 1);
      KDL::Frame f = seg_tf_map.at(rotor);
      rotors_origin_from_target_frame_.at(i) = (target_frame.Inverse() * f).p;
      rotors_normal_from_target_frame_.at(i) = (target_frame.Inverse() * f).M * KDL::Vector(0, 0, 1);
    }

  /* wrench allocation matrix from target frame */
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromTargetFrame<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromTargetFrame<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();
  const auto& sigma = getRotorDirection();
  const double m_f_rate = getMFRate();

  wrench_allocation_matrix_from_target_frame_.resize(6, rotor_num);
  for(int i = 0; i < rotor_num; i++)
    {
      wrench_allocation_matrix_from_target_frame_.block(0, i, 3, 1) = u.at(i);
      wrench_allocation_matrix_from_target_frame_.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
    }

  /* torque by gravity in target frame */
  target_to_cog_frame_ = target_frame.Inverse() * cog;
  cog_to_target_frame_ = cog.Inverse() * target_frame;
  Eigen::Vector3d target_to_cog_p = aerial_robot_model::kdlToEigen(target_to_cog_frame_.p);
  Eigen::MatrixXd skew_mat = aerial_robot_model::skew(target_to_cog_p);
  Eigen::VectorXd gravity = getGravity3d();
  gravity_torque_from_target_frame_ = getMass() * skew_mat * gravity;

}

void RollingRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);

  const auto seg_tf_map = fullForwardKinematics(joint_positions);

  KDL::TreeFkSolverPos_recursive fk_solver(getTree());

  /* get cog */
  KDL::Frame cog = getCog<KDL::Frame>();

  /* link based on COG */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s);
      links_rotation_from_cog_[i] = cog.M.Inverse() * f.M;
    }

  // rotor's y and z axis
  for(int i = 0; i < getRotorNum(); i++)
    {
      std::string rotor = thrust_link_ + std::to_string(i + 1);
      KDL::Frame f = seg_tf_map.at(rotor);
      rotors_x_axis_from_cog_.at(i) = (cog.Inverse() * f).M * KDL::Vector(1, 0, 0);
      rotors_y_axis_from_cog_.at(i) = (cog.Inverse() * f).M * KDL::Vector(0, 1, 0);
    }

  /* center point */
  KDL::Frame link2;
  KDL::Frame link3;
  fk_solver.JntToCart(joint_positions, link2, "link2");
  fk_solver.JntToCart(joint_positions, link3, "link3");
  KDL::Frame center_point;
  center_point_.p.x((link2.p.x() + link3.p.x()) / 3.0);
  center_point_.p.y((link2.p.y() + link3.p.y()) / 3.0);
  center_point_.M = cog.M;

  /* contact point */
  double target_baselink_roll = getCogDesireOrientation<Eigen::Matrix3d>().eulerAngles(0, 1, 2)(0);
  double target_baselink_pitch = getCogDesireOrientation<Eigen::Matrix3d>().eulerAngles(0, 1, 2)(1);
  KDL::Frame cog2baselink = getCog2Baselink<KDL::Frame>();
  KDL::Vector contact_point_offset = KDL::Vector(0, -circle_radius_ * cos(target_baselink_roll), -circle_radius_ * sin(target_baselink_roll));
  // contact_point_.p = cog.p + cog.M * cog2baselink.M * contact_point_offset;
  contact_point_.p = center_point_.p + cog.M * contact_point_offset;
  contact_point_.M = cog.M;

  // publish origin and normal for debug
  geometry_msgs::PoseArray rotor_origin_msg;
  geometry_msgs::PoseArray rotor_normal_msg;
  std::vector<Eigen::Vector3d> rotor_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotor_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  for(int i = 0; i < getRotorNum(); i++)
    {
      geometry_msgs::Pose origin;
      origin.position.x = rotor_origin.at(i)(0);
      origin.position.y = rotor_origin.at(i)(1);
      origin.position.z = rotor_origin.at(i)(2);
      rotor_origin_msg.poses.push_back(origin);
      geometry_msgs::Pose normal;
      normal.position.x = rotor_normal.at(i)(0);
      normal.position.y = rotor_normal.at(i)(1);
      normal.position.z = rotor_normal.at(i)(2);
      rotor_normal_msg.poses.push_back(normal);
    }
  rotor_origin_pub_.publish(rotor_origin_msg);
  rotor_normal_pub_.publish(rotor_normal_msg);

  // feasible wrench
  geometry_msgs::PoseArray feasible_force_array_msg;
  geometry_msgs::PoseArray feasible_torque_array_msg;
  std_msgs::Float32 feasible_force_radius_msg;
  std_msgs::Float32 feasible_torque_radius_msg;

  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  auto f_min_list = calcFeasibleControlFDists(u);
  auto t_min_list = calcFeasibleControlTDists(v);
  // feasible_torque_array_msg.header.frame_id = "unidirection";
  bool t_x_plus_flag = false;
  bool t_x_minus_flag = false;
  bool t_y_plus_flag = false;
  bool t_y_minus_flag = false;
  bool t_z_plus_flag = false;
  bool t_z_minus_flag = false;

  for(const auto& force_u: u)
    {
      geometry_msgs::Pose tmp;
      tmp.position.x = force_u.x();
      tmp.position.y = force_u.y();
      tmp.position.z = force_u.z();
      feasible_force_array_msg.poses.push_back(tmp);
    }
  feasible_force_radius_msg.data = f_min_list.minCoeff();

  for(const auto& torque_v: v)
    {
      geometry_msgs::Pose tmp;
      tmp.position.x = torque_v.x();
      if(torque_v.x() > 0.0) t_x_plus_flag = true;
      if(torque_v.x() < 0.0) t_x_minus_flag = true;
      tmp.position.y = torque_v.y();
      if(torque_v.y() > 0.0) t_y_plus_flag = true;
      if(torque_v.y() < 0.0) t_y_minus_flag = true;
      tmp.position.z = torque_v.z();
      if(torque_v.z() > 0.0) t_z_plus_flag = true;
      if(torque_v.z() < 0.0) t_z_minus_flag = true;
      feasible_torque_array_msg.poses.push_back(tmp);
    }
  if(t_x_plus_flag && t_x_minus_flag && t_y_plus_flag && t_y_minus_flag && t_z_plus_flag && t_z_minus_flag) feasible_torque_radius_msg.data = t_min_list.minCoeff();
  else feasible_torque_radius_msg.data = 0.0;

  feasible_control_torque_pub_.publish(feasible_torque_array_msg);
  feasible_control_torque_radius_pub_.publish(feasible_torque_radius_msg);
  feasible_control_force_pub_.publish(feasible_force_array_msg);
  feasible_control_force_radius_pub_.publish(feasible_force_radius_msg);

}

Eigen::VectorXd RollingRobotModel::calcFeasibleControlFDists(std::vector<Eigen::Vector3d>&u)
{
  u.clear();
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();

  Eigen::Vector3d gravity = getGravity3d();
  std::vector<Eigen::Vector3d> rotor_normal = getRotorsNormalFromCog<Eigen::Vector3d>();

  for(int i = 0; i < rotor_num; i++)
    {
      u.push_back(rotor_normal.at(i));
      // Eigen::Vector3d y_axis_i = kdlToEigen(rotors_coord_rotation_from_cog_.at(i)).col(1);
      // Eigen::Vector3d z_axis_i = kdlToEigen(rotors_coord_rotation_from_cog_.at(i)).col(2);
      // u.push_back(y_axis_i);
      // u.push_back(z_axis_i);
    }

  Eigen::VectorXd fc_f_dists(u.size() * (u.size() - 1) / 2);
  int f_min_index = 0;
  for(int i = 0; i < u.size(); i++)
    {
      for(int j = i + 1; j < u.size(); j++)
        {
          if(i == j) continue;
          double dist_ij = 0.0;
          if(u.at(i).cross(u.at(j)).norm() < 1e-5)
            {
              ROS_DEBUG_NAMED("robot_model", "the direction of u%d and u%d are too close, so no plane can generate by these two vector", i , j);
              dist_ij = 1e6;
            }
          else
            {
              for(int k = 0; k < u.size(); ++k)
                {
                  if(i == k || j == k) continue;
                  double u_triple_product = calcTripleProduct(u.at(i), u.at(j), u.at(k));
                  dist_ij += fabs(u_triple_product * thrust_max);
                }
            }
          Eigen::Vector3d uixuj = u.at(i).cross(u.at(j));
          fc_f_dists(f_min_index) = fabs(dist_ij - (uixuj.dot(gravity) / uixuj.norm()));
          f_min_index++;
        }
    }
  return fc_f_dists;
}


Eigen::VectorXd RollingRobotModel::calcFeasibleControlTDists(std::vector<Eigen::Vector3d>&v)
{
  v.clear();
  const int rotor_num = getRotorNum();
  const auto& sigma = getRotorDirection();
  const double m_f_rate = getMFRate();
  const double thrust_max = getThrustUpperLimit();
  std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  for(int i = 0; i < rotor_num; i++)
    {
      // v.push_back(p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i));
      Eigen::Vector3d y_axis_i = kdlToEigen(rotors_y_axis_from_cog_.at(i));
      Eigen::Vector3d z_axis_i = u.at(i);
      // v.push_back(p.cross(rotors_));
      v.push_back(p.at(i).cross(y_axis_i));
      v.push_back(p.at(i).cross(z_axis_i) + m_f_rate * sigma.at(i + 1) * z_axis_i);
      // v.push_back(p.cross(z_axis_i));
    }

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
