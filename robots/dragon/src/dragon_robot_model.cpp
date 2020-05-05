#include <dragon/dragon_robot_model.h>

DragonRobotModel::DragonRobotModel(bool init_with_rosparam, bool verbose, double epsilon, double control_margin_thre, double wrench_mat_det_thre, double edf_radius, double edf_max_tilt) :
  HydrusRobotModel(init_with_rosparam, verbose, epsilon, 3, control_margin_thre, wrench_mat_det_thre),
  edf_radius_(edf_radius),
  edf_max_tilt_(edf_max_tilt)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  links_rotation_from_cog_.resize(getRotorNum());
  edfs_origin_from_cog_.resize(getRotorNum() * 2);
  gimbal_nominal_angles_.resize(getRotorNum() * 2);

  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      edf_names_.push_back(std::string("edf") + s + std::string("_left"));
      edf_names_.push_back(std::string("edf") + s + std::string("_right"));
    }

  // gimbal jacobian
  const auto& joint_indices = getJointIndices();
  const auto& link_joint_indices = getLinkJointIndices();
  gimbal_jacobian_ = Eigen::MatrixXd::Zero(6 + getJointNum(), 6 + getLinkJointIndices().size());
  gimbal_jacobian_.topLeftCorner(6,6) = Eigen::MatrixXd::Identity(6,6);

  int j = 0;
  for(int i = 0; i < joint_indices.size(); i++)
    {
      if(joint_indices.at(i) == link_joint_indices.at(j))
        {
          gimbal_jacobian_(6 + i, 6 + j) = 1;
          j++;
        }
      if(j == link_joint_indices.size()) break;
    }

  //setCogDesireOrientation(0.0, -0.3, 0);
}

void DragonRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("edf_radius", edf_radius_, 0.035); //70mm EDF
  nhp.param("edf_max_tilt", edf_max_tilt_, 0.26); //15 [deg]
}

bool DragonRobotModel::stabilityCheck(bool verbose)
{
  if(!HydrusRobotModel::stabilityCheck(verbose)) return false;

  /* check the propeller overlap */
  if(!overlapCheck(verbose))
    {
      ROS_ERROR("propeller overlapped!");
      return false;
    }

  return true;
}

bool DragonRobotModel::overlapCheck(bool verbose)
{
  const std::vector<Eigen::Vector3d> edfs_origin_from_cog = getEdfsOriginFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  for(int i = 0; i < rotor_num * 2; ++i)
    {
      for(int j = i + 1; j < rotor_num * 2; ++j)
        {
          /* special for dual rotor */
          if(i / 2 == j / 2) continue;

          Eigen::Vector3d diff = edfs_origin_from_cog.at(i) - edfs_origin_from_cog.at(j); //dual
          double projected_dist = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
          //approximated, the true one should be (edf_radius_ / cos(tilt) + diff(2) * tan(tilt)
          double dist_thre = edf_radius_ + fabs(diff(2)) * tan(edf_max_tilt_) + edf_radius_;

          /* debug */
          if(dist_thre > projected_dist)
            {
              ROS_ERROR_STREAM(edf_names_.at(i) << " and " << edf_names_.at(j) << " is overlapped");
              return false;
            }
        }
    }
  return true;
}

void DragonRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  /*
     Although we can use "aerial_robot_model::forwardKinematicsImpl()",
     "TreeFkSolverPos_recursive::TreeFkSolverPos_recursive(const Tree& _tree)"
     takes quite "long" time to create new tree as class member.

     please refer to:
     - https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/treefksolverpos_recursive.cpp#L28-L31
     - https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/tree.cpp#L34-L42
  */
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
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s);

      links_rotation_from_cog_[i] = cog_frame.Inverse() * f.M;
      double r, p, y;
      links_rotation_from_cog_[i].GetRPY(r, p, y);

      gimbal_processed_joint_(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = -r;
      gimbal_processed_joint_(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = -p;

      gimbal_nominal_angles_[i * 2] = -r;
      gimbal_nominal_angles_[i * 2 + 1] = -p;
    }

  /* normal robot model update */
  HydrusRobotModel::updateRobotModelImpl(gimbal_processed_joint_);

  /* special process for dual edf gimbal */
  /* set the edf position w.r.t CoG frame */
  const auto& seg_frames = getSegmentsTf();
  std::vector<KDL::Vector> f_edfs;
  for(const auto& name: edf_names_)
    f_edfs.push_back((getCog<KDL::Frame>().Inverse() * seg_frames.at(name)).p);

  edfs_origin_from_cog_ = f_edfs;
}

Eigen::MatrixXd DragonRobotModel::getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset)
{
  Eigen::MatrixXd jacobian = aerial_robot_model::RobotModel::getJacobian(joint_positions, segment_name, offset);
  return jacobian * gimbal_jacobian_;
}

void DragonRobotModel::calcBasicKinematicsJacobian()
{
  aerial_robot_model::RobotModel::calcBasicKinematicsJacobian();

  const auto& joint_names = getJointNames();
  const auto& link_joint_names = getLinkJointNames();
  const auto& link_joint_indices = getLinkJointIndices();
  const auto& joint_index_map = getJointIndexMap();

  std::vector<Eigen::MatrixXd> thrust_coord_jacobians = getThrustCoordJacobians();

  // the rotor normal of gimbal is always vertial
  for(int i = 0; i < getRotorNum(); i++)
    {
      Eigen::MatrixXd joint_rot_jacobian = Eigen::MatrixXd::Zero(3, 6 + link_joint_indices.size());

      joint_rot_jacobian.leftCols(6) = thrust_coord_jacobians.at(i).bottomLeftCorner(3, 6); // root
      std::string gimbal = std::string("gimbal") + std::to_string(i + 1);
      int gimbal_roll_index, gimbal_pitch_index;
      int j = 0;
      for(int k = 0; k < joint_names.size(); k++)
        {
          if(j < link_joint_names.size())
            {
              if(joint_names.at(k) == link_joint_names.at(j))
                {
                  joint_rot_jacobian.col(6 + j) = thrust_coord_jacobians.at(i).block(3, 6 + k, 3, 1);
                  j++;
                }
            }
          if(joint_names.at(k) == gimbal + std::string("_roll")) gimbal_roll_index = 6 + k;
          if(joint_names.at(k) == gimbal + std::string("_pitch")) gimbal_pitch_index = 6 + k;
        }

       // ROS_ERROR_STREAM("thrust coord jacobian: \n" << thrust_coord_jacobians.at(i));
       // ROS_ERROR_STREAM("joint rot jacobian: \n" << joint_rot_jacobian);

       // ROS_INFO("roll & pitch index: %d, %d", gimbal_roll_index, gimbal_pitch_index);

      Eigen::MatrixXd gimbal_rot_jacobian = Eigen::MatrixXd::Zero(3, 2);
      gimbal_rot_jacobian.col(0) = thrust_coord_jacobians.at(i).block(3, gimbal_roll_index, 3, 1);
      gimbal_rot_jacobian.col(1) = thrust_coord_jacobians.at(i).block(3, gimbal_pitch_index, 3, 1);

      //ROS_INFO_STREAM("gimbal rot jacobian  for rotor" << i+1 << ": \n" << gimbal_rot_jacobian);

      Eigen::MatrixXd gimbal_joint_jacobian = - (Eigen::MatrixXd::Identity(2,3) * gimbal_rot_jacobian).inverse() * Eigen::MatrixXd::Identity(2,3) * joint_rot_jacobian;

      //ROS_INFO_STREAM("gimbal joint jacobian for rotor" << i+1 << ": \n" << gimbal_joint_jacobian);

      gimbal_jacobian_.row(gimbal_roll_index) = gimbal_joint_jacobian.row(0);
      gimbal_jacobian_.row(gimbal_pitch_index) = gimbal_joint_jacobian.row(1);
    }
  //ROS_ERROR_STREAM("dragon gimbal jacobian: \n" << gimbal_jacobian_);
}

void DragonRobotModel::calcCoGMomentumJacobian()
{
  setCOGJacobian(Eigen::MatrixXd::Zero(3, 6 + getJointNum()));
  setLMomentumJacobian(Eigen::MatrixXd::Zero(3, 6 + getJointNum()));

  aerial_robot_model::RobotModel::calcCoGMomentumJacobian();
}

void DragonRobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model)
{
  if(update_model) updateRobotModel(joint_positions);

  calcBasicKinematicsJacobian();

  calcCoGMomentumJacobian();

  calcLambdaJacobian();

  calcJointTorque();

  calcJointTorqueJacobian();

  // convert to jacobian only for link joint
  std::vector<Eigen::MatrixXd> u_jacobians = getUJacobians();
  for(auto& jacobian: u_jacobians)
    {
      //ROS_INFO_STREAM("prev u jacobian: \n" << jacobian);
      jacobian = jacobian * gimbal_jacobian_;
    }
  setUJacobians(u_jacobians);

  std::vector<Eigen::MatrixXd> p_jacobians = getPJacobians();
  for(auto& jacobian: p_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setPJacobians(p_jacobians);

  std::vector<Eigen::MatrixXd> v_jacobians = getVJacobians();
  for(auto& jacobian: v_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setVJacobians(v_jacobians);

  std::vector<Eigen::MatrixXd> thrust_coord_jacobians = getThrustCoordJacobians();
  for(auto& jacobian: thrust_coord_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setThrustTCoordJacobians(thrust_coord_jacobians);

  std::vector<Eigen::MatrixXd> cog_coord_jacobians = getCOGCoordJacobians();
  for(auto& jacobian: cog_coord_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setCOGCoordJacobians(cog_coord_jacobians);

  Eigen::MatrixXd cog_jacobian = getCOGJacobian();
  setCOGJacobian(cog_jacobian * gimbal_jacobian_);
  Eigen::MatrixXd l_momentum_jacobian = getLMomentumJacobian();
  setLMomentumJacobian(l_momentum_jacobian * gimbal_jacobian_);

  Eigen::MatrixXd lambda_jacobian = getLambdaJacobian();
  setLambdaJacobian(lambda_jacobian * gimbal_jacobian_);
  Eigen::MatrixXd joint_torque_jacobian = getJointTorqueJacobian();
  setJointTorqueJacobian(joint_torque_jacobian * gimbal_jacobian_);


  /* update jacobian for rotor overlap */
  calcRotorOverlapJacobian();

  thrustForceNumericalJacobian(getJointPositions(), getLambdaJacobian(), getLinkJointIndices());

  jointTorqueNumericalJacobian(getJointPositions(), getJointTorqueJacobian(), getLinkJointIndices());
  cogMomentumNumericalJacobian(getJointPositions(), getCOGJacobian(), getLMomentumJacobian(), getLinkJointIndices());

  // overlapNumericalJacobian(getJointPositions(), rotor_overlap_jacobian_);

  //throw std::runtime_error("test");
}


void DragonRobotModel::thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
{
  aerial_robot_model::RobotModel::thrustForceNumericalJacobian(joint_positions, analytical_result, joint_indices);
}


void DragonRobotModel::calcRotorOverlapJacobian()
{
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::MatrixXd>& u_jacobians = getUJacobians();

  const int rotor_num = getRotorNum();

  min_dist_squared_ = 1e6;

  for(int i = 0; i < rotor_num * 2; ++i)
    {
      for(int j = i + 1; j < rotor_num * 2; ++j)
        {
          if(i / 2 == j / 2) continue;

          bool swap = false;
          if(edfs_origin_from_cog_.at(i).z() < edfs_origin_from_cog_.at(j).z())
            {
              swap = true;
              std::swap(i, j);
            }

          Eigen::Vector3d p_i = aerial_robot_model::kdlToEigen(edfs_origin_from_cog_.at(i));
          Eigen::Vector3d p_j = aerial_robot_model::kdlToEigen(edfs_origin_from_cog_.at(j));
          Eigen::Vector3d p_v = p_i + (p_j(2) - p_i(2)) * u.at(i/2) / u.at(i/2)(2);

          if((p_v - p_j).squaredNorm() < min_dist_squared_)
            {
              min_dist_squared_ = (p_v - p_j).squaredNorm();
              rotor_i_ = i;
              rotor_j_ = j;
              p_i_ = p_i;
              p_j_ = p_j;
              p_v_ = p_v;
              // ROS_INFO("%d, %d, %f", rotor_i_, rotor_j_, min_dist_squared_);
            }

          if(swap) std::swap(i, j);
        }
    }

  // ROS_INFO_STREAM("ovverlap: i:" << edf_names_.at(rotor_i_) << "; j:" << edf_names_.at(rotor_j_));
  Eigen::MatrixXd p_i_jacobian = getJacobian(getJointPositions(), edf_names_.at(rotor_i_)).topRows(3);
  Eigen::MatrixXd p_j_jacobian = getJacobian(getJointPositions(), edf_names_.at(rotor_j_)).topRows(3);
  Eigen::MatrixXd p_v_jacobian = p_i_jacobian + (p_j_(2) - p_i_(2)) / u.at(rotor_i_/2)(2) * u_jacobians.at(rotor_i_/2) + u.at(rotor_i_/2) / u.at(rotor_i_/2)(2) * (p_j_jacobian.row(2) - p_i_jacobian.row(2)) - u.at(rotor_i_/2) / std::pow(u.at(rotor_i_/2)(2), 2) * (p_j_(2) - p_i_(2)) * u_jacobians.at(rotor_i_/2).row(2);
  rotor_overlap_jacobian_ = 2 * (p_v_ - p_j_).transpose() * (p_v_jacobian - p_j_jacobian);
}

void DragonRobotModel::overlapNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result)
{
  const std::map<std::string, KDL::Frame> seg_frames = getSegmentsTf();
  const int rotor_num = getRotorNum();
  KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse();
  const std::vector<KDL::Vector> nominal_edfs_origin_from_cog = edfs_origin_from_cog_;

  Eigen::MatrixXd J_overlap = Eigen::MatrixXd::Zero(1, 6 + getLinkJointIndices().size());

  double delta_angle = 0.00001; // [rad]
  int col_index = 6;

  auto perturbationJointTorque = [&](int col, KDL::JntArray joint_angles)
    {
      updateRobotModelImpl(joint_angles);
      auto diff = edfs_origin_from_cog_.at(rotor_i_) - edfs_origin_from_cog_.at(rotor_j_);
      diff.z(0);
      J_overlap(0, col) = (std::pow(diff.Norm(), 2) - min_dist_squared_) / delta_angle;
    };

  for (const auto& joint_index : getLinkJointIndices()) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    updateRobotModelImpl(perturbation_joint_positions);
    setCogDesireOrientation(root_rot * getSegmentsTf().at(getBaselinkName() ).M);
    perturbationJointTorque(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(getBaselinkName()).M);
  perturbationJointTorque(3, joint_positions);

  // pitch
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(getBaselinkName()).M);
  perturbationJointTorque(4, joint_positions);

  // yaw
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(getBaselinkName()).M);
  perturbationJointTorque(5, joint_positions);

  // reset
  setCogDesireOrientation(baselink_rot); // set the orientation of root
  updateRobotModelImpl(joint_positions);

  ROS_INFO_STREAM("numerical result of rotor overlap jacobian: \n" << J_overlap);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_INFO_STREAM("analytical_result: \n" << analytical_result);
      ROS_INFO_STREAM("diff of  jacobian: \n" << J_overlap - analytical_result);

      double min_diff = (J_overlap - analytical_result).minCoeff();
      double max_diff = (J_overlap - analytical_result).maxCoeff();

      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of overlap jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of overlap jacobian: " << fabs(min_diff));
    }
}
