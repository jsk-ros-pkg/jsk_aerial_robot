#include <delta/model/delta_robot_model.h>

DeltaRobotModel::DeltaRobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre,
                                 double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  rotor_on_rigid_frame_num_ = 4;
  rotor_on_soft_frame_num_ = rotor_num - rotor_on_rigid_frame_num_;
  links_rotation_from_cog_.resize(rotor_on_rigid_frame_num_);
  // current_joint_angles_.resize(getJointNum() - rotor_num);
  current_gimbal_angles_.resize(rotor_on_rigid_frame_num_);

  // note: it might be better to use gimbal_link5
  ros::NodeHandle nh;
  rotor5_pose_sub_ = nh.subscribe("thrust5/mocap/pose", 1, &DeltaRobotModel::Rotor5MocapCallback, this);
  body_pose_sub_ = nh.subscribe("mocap/pose", 1, &DeltaRobotModel::BodyMocapCallback, this);

}

void DeltaRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  auto start = std::chrono::high_resolution_clock::now();

  RobotModel::updateRobotModelImpl(joint_positions);

  const auto& seg_tf_map = getSegmentsTf();
  const auto& joint_index_map = getJointIndexMap();

  KDL::Frame cog = getCog<KDL::Frame>();

  /* link and rotor information in each link */
  for (int i = 0; i < rotor_on_rigid_frame_num_; ++i)
  {
    std::string s = std::to_string(i + 1);

    /* link */
    KDL::Frame link_f = seg_tf_map.at("link" + s);
    links_rotation_from_cog_.at(i) = (cog.Inverse() * link_f).M;

    /* get current gimbal angles */
    current_gimbal_angles_.at(i) =
        joint_positions(joint_index_map.find(std::string("gimbal") + std::to_string(i + 1))->second);
  }

  /* get current joint angles */
  // {
  //   current_joint_angles_.at(i) =
  //       joint_positions(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second);
  // }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}

Eigen::MatrixXd DeltaRobotModel::getFullWrenchAllocationMatrixFromCog()
{
  /* calculate normal allocation */
  Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * rotor_on_rigid_frame_num_);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  /* get cog */
  KDL::Frame cog = getCog<KDL::Frame>();

  int last_col = 0;
  std::vector<KDL::Vector> rotors_origin_from_cog = getRotorsOriginFromCog<KDL::Vector>();
  std::map<int, int> rotor_direction = getRotorDirection();
  for (int i = 0; i < rotor_on_rigid_frame_num_; i++)
  {
    double m_f_rate = getMFRate(i);
    wrench_map.block(3, 0, 3, 3) =
          aerial_robot_model::skew(kdlToEigen(rotors_origin_from_cog.at(i))) +
          m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3);  // [p x] + sigma E_3
    wrench_matrix.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  /* calculate masked and integrated rotaion matrix */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * rotor_on_rigid_frame_num_, 2 * rotor_on_rigid_frame_num_);
  Eigen::MatrixXd mask(3, 2);  // y and z axis
  mask << 0, 0, 1, 0, 0, 1;
  for (int i = 0; i < rotor_on_rigid_frame_num_; i++)
  {
    integrated_rot.block(3 * i, 2 * i, 3, 2) = kdlToEigen(links_rotation_from_cog_.at(i)) * mask;
  }

  /* calculate integarated allocation */
  Eigen::MatrixXd full_q_mat_for_rotors_on_rigid_frame = wrench_matrix * integrated_rot;
  Eigen::MatrixXd q_mat_for_rotors_on_soft_frame = getQMatForRotorsOnSoftFrame();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 2 * rotor_on_rigid_frame_num_ + rotor_on_soft_frame_num_);
  full_q_mat.block(0, 0, 6, 2 * rotor_on_rigid_frame_num_) = full_q_mat_for_rotors_on_rigid_frame;
  full_q_mat.block(0, 2 * rotor_on_rigid_frame_num_, 6, rotor_on_soft_frame_num_) = q_mat_for_rotors_on_soft_frame;
  return full_q_mat;
}

Eigen::MatrixXd DeltaRobotModel::getQMatForRotorsOnSoftFrame()
{
  // wrench allocation matrix
  std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  auto& rotor_direction = getRotorDirection();

  if (ros::Time::now().toSec() - rotor5_pose_update_time_.toSec() < 1.0 && 
      ros::Time::now().toSec() - body_pose_update_time_.toSec() < 1.0){
    KDL::Frame body_pose_from_root_ = getSegmentsTf().at("fc");
    KDL::Frame rotor5_pose_from_root = body_pose_from_root_ * body_pose_from_world_.Inverse() * rotor5_pose_from_world_;
    KDL::Frame cog = getCog<KDL::Frame>();
    rotors_origin.at(4) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor5_pose_from_root).p);
    rotors_normal.at(4) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor5_pose_from_root).M * KDL::Vector(0,0,1));
  } else {
    std::cout << "Warning: lost mocap for rotor5" << std::endl;
    rotors_origin.at(4) = prev_rotor5_origin;
    rotors_normal.at(4) = prev_rotor5_normal;
  }

  // fail safe for mocap update
  if (prev_rotor5_origin != Eigen::Vector3d(0,0,0) && prev_rotor5_normal != Eigen::Vector3d(0,0,0)){
      for (unsigned int i = 0; i < 3; ++i) {
      if (abs(rotors_origin.at(4)(i) - prev_rotor5_origin(i)) > 0.5 || abs(rotors_normal.at(4)(i) - prev_rotor5_normal(i)) > 0.5){
        rotors_origin.at(4) = prev_rotor5_origin;
        rotors_normal.at(4) = prev_rotor5_normal;
        std::cout << "fail safe for mocap update!!!!" << std::endl;
        break;
      }
    }
  }

  // low pass filter for rotor5 pose and orientation
  if (prev_rotor5_origin != Eigen::Vector3d(0,0,0) && prev_rotor5_normal != Eigen::Vector3d(0,0,0)){
    double alpha = 0.5;
    rotors_origin.at(4) = alpha * prev_rotor5_origin + (1 - alpha) * rotors_origin.at(4);
    rotors_normal.at(4) = alpha * prev_rotor5_normal + (1 - alpha) * rotors_normal.at(4);
    rotors_normal.at(4).normalize();
  }

  prev_rotor5_origin = rotors_origin.at(4);
  prev_rotor5_normal = rotors_normal.at(4);
  
  Eigen::MatrixXd q_mat_for_rotors_on_soft_frame = Eigen::MatrixXd::Zero(6, rotor_on_soft_frame_num_);
  for (int i = rotor_on_rigid_frame_num_; i < getRotorNum(); i++)
  {
    double m_f_rate = getMFRate(i);
    q_mat_for_rotors_on_soft_frame(0, i-rotor_on_rigid_frame_num_) = rotors_normal.at(i).x();
    q_mat_for_rotors_on_soft_frame(1, i-rotor_on_rigid_frame_num_) = rotors_normal.at(i).y();
    q_mat_for_rotors_on_soft_frame(2, i-rotor_on_rigid_frame_num_) = rotors_normal.at(i).z();
    q_mat_for_rotors_on_soft_frame.block(3, i-rotor_on_rigid_frame_num_, 3, 1) = (rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
  }
  return q_mat_for_rotors_on_soft_frame;
}

void DeltaRobotModel::Rotor5MocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, rotor5_pose_from_world_);
  rotor5_pose_update_time_ = ros::Time::now();
}

void DeltaRobotModel::BodyMocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, body_pose_from_world_);
  body_pose_update_time_ = ros::Time::now();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(DeltaRobotModel, aerial_robot_model::RobotModel);
