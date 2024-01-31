#include <delta/model/delta_robot_model.h>

RollingRobotModel::RollingRobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  links_position_from_cog_.resize(rotor_num);
  links_rotation_from_cog_.resize(rotor_num);
  links_center_frame_from_cog_.resize(rotor_num);
  thrust_link_ = "thrust";

  ros::NodeHandle nh;
  feasible_control_force_pub_  = nh.advertise<geometry_msgs::PoseArray>("feasible_control_force_convex", 1);
  feasible_control_torque_pub_ = nh.advertise<geometry_msgs::PoseArray>("feasible_control_torque_convex", 1);
  feasible_control_force_radius_pub_  = nh.advertise<std_msgs::Float32>("feasible_control_force_radius", 1);
  feasible_control_torque_radius_pub_ = nh.advertise<std_msgs::Float32>("feasible_control_torque_radius", 1);
  rotor_origin_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug/rotor_origin", 1);
  rotor_normal_pub_ = nh.advertise<geometry_msgs::PoseArray>("debug/rotor_normal", 1);

  target_frame_name_ = "cog";
  additional_frame_["cp"] = &contact_point_;
  additional_frame_["contact_point_real"] = &contact_point_real_;
  setTargetFrame(target_frame_name_);
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
      KDL::Frame f, f_from_cog;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s);
      f_from_cog = cog.Inverse() * f;
      links_position_from_cog_.at(i) = f_from_cog.p;
      links_rotation_from_cog_.at(i) = f_from_cog.M;
    }

  /* circle center of each links */
  for(int i = 0; i < getRotorNum(); i++)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s + std::string("_center"));
      links_center_frame_from_cog_.at(i) = cog.Inverse() * f;
    }

  /* calculate inertia from root */
  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  std::map<std::string, KDL::RigidBodyInertia> inertia_map = RobotModel::getInertiaMap();
  for(const auto& inertia : inertia_map)
    {
      KDL::Frame f = seg_tf_map.at(inertia.first);
      link_inertia = link_inertia + f * inertia.second;
    }
  link_inertia_ = link_inertia;

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
  contact_point_.p = center_point_.p + cog.M * contact_point_offset;
  contact_point_.M = cog.M;

  /* publish origin and normal for debug */
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
}

void RollingRobotModel::setTargetFrame(std::string frame_name)
{
  KDL::Frame frame;
  const std::map<std::string, KDL::Frame> seg_tf_map = getSegmentsTf();
  if(frame_name == "cog")
    {
      frame = getCog<KDL::Frame>();
      setTargetFrame(frame);
      target_frame_name_ = frame_name;
      return;
    }

  if(seg_tf_map.find(frame_name) == seg_tf_map.end() && additional_frame_.find(frame_name) == additional_frame_.end())
    {
      ROS_ERROR_STREAM("[model] there is not frame named " << frame_name);
      return;
    }
  else
    {
      target_frame_name_ = frame_name;
      if(seg_tf_map.find(target_frame_name_) == seg_tf_map.end())
        {
          frame = *(additional_frame_.at(target_frame_name_));
        }
      else
        {
          frame = seg_tf_map.at(target_frame_name_);
        }
      setTargetFrame(frame);
      return;
    }
}

Eigen::MatrixXd RollingRobotModel::getFullWrenchAllocationMatrixFromControlFrame()
{
  /* calculate normal allocation */
  const int rotor_num = getRotorNum();
  Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * rotor_num);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);

  /* get cog */
  KDL::Frame cog = getCog<KDL::Frame>();

  int last_col = 0;
  std::vector<KDL::Vector> rotors_origin_from_cog = getRotorsOriginFromCog<KDL::Vector>();
  for(int i = 0; i < rotor_num; i++)
    {
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(kdlToEigen(getTargetFrame().Inverse() * cog * rotors_origin_from_cog.at(i)));
      wrench_matrix.middleCols(last_col, 3) = wrench_map;
      last_col += 3;
    }

  Eigen::Matrix3d inertia_inv = getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse();
  double mass_inv = 1 / getMass();
  wrench_matrix.topRows(3) = mass_inv * wrench_matrix.topRows(3);
  wrench_matrix.bottomRows(3) = inertia_inv * wrench_matrix.bottomRows(3);

  /* calculate masked and integrated rotaion matrix */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * rotor_num, 2 * rotor_num);
  Eigen::MatrixXd mask(3, 2);
  mask << 0, 0, 1, 0, 0, 1;
  for(int i = 0; i < rotor_num; i++)
    {
      integrated_rot.block(3 * i, 2 * i, 3, 2) = kdlToEigen(getTargetFrame().M.Inverse() * cog.M * links_rotation_from_cog_.at(i)) * mask;
    }

  /* calculate integarated allocation */
  Eigen::MatrixXd full_q_mat = wrench_matrix * integrated_rot;
  return full_q_mat;
}

Eigen::MatrixXd RollingRobotModel::getFullWrenchAllocationMatrixFromControlFrame(std::string frame_name)
{
  std::string prev_target_frame_name = getTargetFrameName();
  if(prev_target_frame_name == frame_name)
    {
      return getFullWrenchAllocationMatrixFromControlFrame();
    }
  setTargetFrame(frame_name);
  Eigen::MatrixXd full_q_mat = getFullWrenchAllocationMatrixFromControlFrame();
  setTargetFrame(prev_target_frame_name);
  return full_q_mat;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RollingRobotModel, aerial_robot_model::RobotModel);
