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
  rotor6_pose_sub_ = nh.subscribe("thrust6/mocap/pose", 1, &DeltaRobotModel::Rotor6MocapCallback, this);
  body_pose_sub_ = nh.subscribe("mocap/pose", 1, &DeltaRobotModel::BodyMocapCallback, this);

  rotor_on_soft_frame_pose_from_world_.resize(rotor_on_soft_frame_num_);
  rotor_on_soft_frame_pose_update_time_.resize(rotor_on_soft_frame_num_);

  prev_rotor_on_soft_frame_origin.resize(rotor_on_soft_frame_num_);
  prev_rotor_on_soft_frame_normal.resize(rotor_on_soft_frame_num_);
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

Eigen::MatrixXd DeltaRobotModel::calcWrenchMatrixOnCoG()
{
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();

  //Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  for (unsigned int i = 0; i < rotor_on_rigid_frame_num_; ++i) {
    double m_f_rate = getMFRate(i);
    Q.block(0, i, 3, 1) = u.at(i);
    Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
  }

  Eigen::MatrixXd q_mat_for_rotors_on_soft_frame = getQMatForRotorsOnSoftFrame();
  Q.block(0, rotor_on_rigid_frame_num_, 6, rotor_on_soft_frame_num_) = q_mat_for_rotors_on_soft_frame;

  return Q;
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

  for (int i = rotor_on_rigid_frame_num_; i < getRotorNum(); i++)
  {
    if(ros::Time::now().toSec() - rotor_on_soft_frame_pose_update_time_.at(i - rotor_on_rigid_frame_num_).toSec() < 1.0 && 
        ros::Time::now().toSec() - body_pose_update_time_.toSec() < 1.0){
      KDL::Frame body_pose_from_root_ = getSegmentsTf().at("fc");
      KDL::Frame rotor_pose_from_root = body_pose_from_root_ * body_pose_from_world_.Inverse() * rotor_on_soft_frame_pose_from_world_.at(i - rotor_on_rigid_frame_num_);
      KDL::Frame cog = getCog<KDL::Frame>();
      rotors_origin.at(i) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor_pose_from_root).p);
      rotors_normal.at(i) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor_pose_from_root).M * KDL::Vector(0,0,1));
    } else {
      std::cout << "Warning: lost mocap for rotor" << i + 1 << std::endl;
      int origin_queue_size = prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).size();
      int normal_queue_size = prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).size();
      for (int j = 0; j < origin_queue_size; j++){
        rotors_origin.at(i) = prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).front();
        prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).push(prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).front());
        prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).pop();
      }
      rotors_origin.at(i) /= origin_queue_size;
      for (int j = 0; j < normal_queue_size; j++){
        rotors_normal.at(i) = prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).front();
        prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).push(prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).front());
        prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).pop();
      }
    }

    // fail safe for mocap update
    // if (prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_ ) != Eigen::Vector3d(0,0,0) && 
    //     prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_) != Eigen::Vector3d(0,0,0)){
    //     for (unsigned int j = 0; j < 3; ++j) {
    //     if (abs(rotors_origin.at(i)(j) - prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_)(j)) > 0.5 || 
    //         abs(rotors_normal.at(i)(j) - prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_)(j)) > 0.5){
    //       rotors_origin.at(i) = prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_);
    //       rotors_normal.at(i) = prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_);
    //       std::cout << "fail safe for mocap update!!!!" << std::endl;
    //       break;
    //         }
    //       }
    //     }

    prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).push(rotors_origin.at(i));
    prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).push(rotors_normal.at(i));
    if (prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).size() > 10){
      prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).pop();
    }
    if (prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).size() > 10){
      prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).pop();
    }

    // low pass filter using moving average
    rotors_origin.at(i) = Eigen::Vector3d(0,0,0);
    rotors_normal.at(i) = Eigen::Vector3d(0,0,0);
    int origin_queue_size = prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).size();
    int normal_queue_size = prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).size();
    for (int j = 0; j < origin_queue_size; j++){
      rotors_origin.at(i) += prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).front();
      prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).push(prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).front());
      prev_rotor_on_soft_frame_origin.at(i - rotor_on_rigid_frame_num_).pop();
    }
    rotors_origin.at(i) /= origin_queue_size;
    for (int j = 0; j < normal_queue_size; j++){
      rotors_normal.at(i) += prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).front();
      prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).push(prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).front());
      prev_rotor_on_soft_frame_normal.at(i - rotor_on_rigid_frame_num_).pop();
    }
    rotors_normal.at(i) /= normal_queue_size;
    rotors_normal.at(i).normalize();
  }
  
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
  tf2::fromMsg(msg.pose, rotor_on_soft_frame_pose_from_world_.at(0));
  rotor_on_soft_frame_pose_update_time_.at(0) = ros::Time::now();
}

void DeltaRobotModel::Rotor6MocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, rotor_on_soft_frame_pose_from_world_.at(1));
  rotor_on_soft_frame_pose_update_time_.at(1) = ros::Time::now();
}

void DeltaRobotModel::BodyMocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, body_pose_from_world_);
  body_pose_update_time_ = ros::Time::now();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(DeltaRobotModel, aerial_robot_model::RobotModel);
