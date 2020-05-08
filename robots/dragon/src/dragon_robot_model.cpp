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

  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
  edfs_origin_from_cog_.resize(rotor_num * 2);
  gimbal_nominal_angles_.resize(rotor_num * 2);

  for(int i = 0; i < rotor_num; ++i)
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

  // external wrench
  vectoring_q_mat_ = Eigen::MatrixXd::Zero(6, rotor_num * 3); // init
  vectoring_thrust_ = Eigen::VectorXd::Zero(rotor_num * 3); // init
  wrench_comp_thrust_ = Eigen::VectorXd::Zero(rotor_num * 3); // init
  //setCogDesireOrientation(0.0, -0.3, 0);


  // test
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  wrench(0) = 0; // x
  wrench(1) = 0; // x
  wrench(2) = 1; // z
  addExternalStaticWrench("force1", "link4", KDL::Vector(0.424, 0, 0), wrench);
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

  // external wrench realted jacobians
  calcExternalWrenchCompThrust();
  calcCompThrustJacobian();

  addCompThrustToLambdaJacobian();
  addCompThrustToJointTorqueJacobian();


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

  Eigen::MatrixXd comp_thrust_jacobian = getCompThrustJacobian();
  setCompThrustJacobian(comp_thrust_jacobian * gimbal_jacobian_);

  // update jacobian for rotor overlap
  calcRotorOverlapJacobian();


  compThrustNumericalJacobian(getJointPositions(), comp_thrust_jacobian_);

  thrustForceNumericalJacobian(getJointPositions(), getLambdaJacobian(), getLinkJointIndices());
  jointTorqueNumericalJacobian(getJointPositions(), getJointTorqueJacobian(), getLinkJointIndices());
  // cogMomentumNumericalJacobian(getJointPositions(), getCOGJacobian(), getLMomentumJacobian(), getLinkJointIndices());

  //overlapNumericalJacobian(getJointPositions(), rotor_overlap_jacobian_);

  throw std::runtime_error("test");
}

void DragonRobotModel::calcRotorOverlapJacobian()
{
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::MatrixXd>& u_jacobians = getUJacobians();

  const int rotor_num = getRotorNum();

  min_dist_ = 1e6;

  Eigen::Vector3d closest_p_i, closest_p_j, closest_p_v;

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

          double dist = (p_v - p_j).norm() - (p_i(2) - p_j(2)) * tan(edf_max_tilt_);

          if(dist < min_dist_)
            {
              min_dist_ = dist;
              rotor_i_ = i;
              rotor_j_ = j;
              closest_p_i = p_i;
              closest_p_j = p_j;
              closest_p_v = p_v;
            }

          if(swap) std::swap(i, j);
        }
    }

  // ROS_INFO_STREAM("overlap: i:" << edf_names_.at(rotor_i_) << "; j:" << edf_names_.at(rotor_j_));
  Eigen::MatrixXd p_i_jacobian = getJacobian(getJointPositions(), edf_names_.at(rotor_i_)).topRows(3);
  Eigen::MatrixXd p_j_jacobian = getJacobian(getJointPositions(), edf_names_.at(rotor_j_)).topRows(3);
  Eigen::MatrixXd p_v_jacobian = p_i_jacobian + (closest_p_j(2) - closest_p_i(2)) / u.at(rotor_i_/2)(2) * u_jacobians.at(rotor_i_/2) + u.at(rotor_i_/2) / u.at(rotor_i_/2)(2) * (p_j_jacobian.row(2) - p_i_jacobian.row(2)) - u.at(rotor_i_/2) / std::pow(u.at(rotor_i_/2)(2), 2) * (closest_p_j(2) - closest_p_i(2)) * u_jacobians.at(rotor_i_/2).row(2);
  rotor_overlap_jacobian_ = (closest_p_v - closest_p_j).transpose() * (p_v_jacobian - p_j_jacobian) / (closest_p_v - closest_p_j).norm();
  rotor_overlap_jacobian_ -= tan(edf_max_tilt_) * (p_i_jacobian.row(2) - p_j_jacobian.row(2));
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

  auto perturbation = [&](int col, KDL::JntArray joint_angles)
    {
      updateRobotModelImpl(joint_angles);
      auto diff = edfs_origin_from_cog_.at(rotor_i_) - edfs_origin_from_cog_.at(rotor_j_);
      double d_tilt = diff.z() * tan(edf_max_tilt_);
      diff.z(0);
      J_overlap(0, col) = (diff.Norm() - d_tilt - min_dist_) / delta_angle;
    };

  for (const auto& joint_index : getLinkJointIndices()) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    updateRobotModelImpl(perturbation_joint_positions);
    setCogDesireOrientation(root_rot * getSegmentsTf().at(getBaselinkName() ).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(getBaselinkName()).M);
  perturbation(3, joint_positions);

  // pitch
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(getBaselinkName()).M);
  perturbation(4, joint_positions);

  // yaw
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(getBaselinkName()).M);
  perturbation(5, joint_positions);

  // reset
  setCogDesireOrientation(baselink_rot); // set the orientation of root
  updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical result of rotor overlap jacobian: \n" << J_overlap);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_DEBUG_STREAM("analytical_result: \n" << analytical_result);
      ROS_DEBUG_STREAM("diff of  jacobian: \n" << J_overlap - analytical_result);

      double min_diff = (J_overlap - analytical_result).minCoeff();
      double max_diff = (J_overlap - analytical_result).maxCoeff();

      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of overlap jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of overlap jacobian: " << fabs(min_diff));
    }
}

std::vector<int> DragonRobotModel::getClosestRotorIndices()
{
  std::vector<int> rotor_indices;
  rotor_indices.push_back(rotor_i_);
  rotor_indices.push_back(rotor_j_);
  return rotor_indices;
}

bool DragonRobotModel::addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const KDL::Vector offset, const Eigen::VectorXd wrench)
{
  if(external_wrench_map_.find(wrench_name) != external_wrench_map_.end())
    {
      ROS_WARN_STREAM(wrench_name << " is already in the wrench map.");
      return false;
    }

  external_wrench_map_[wrench_name] =  ExternalWrench{reference_frame, offset, wrench};

  return true;
}

bool DragonRobotModel::addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const geometry_msgs::Point offset, const geometry_msgs::Wrench wrench)
{
  KDL::Vector offset_kdl;
  tf::pointMsgToKDL(offset, offset_kdl);
  Eigen::Matrix<double, 6, 1> wrench_eigen;
  tf::wrenchMsgToEigen(wrench, wrench_eigen);

  return addExternalStaticWrench(wrench_name, reference_frame, offset_kdl, wrench_eigen);
}

bool DragonRobotModel::addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const KDL::Vector offset, const KDL::Wrench wrench)
{
  Eigen::VectorXd wrench_eigen(6);
  wrench_eigen.head(3) = aerial_robot_model::kdlToEigen(wrench.force);
  wrench_eigen.tail(3) = aerial_robot_model::kdlToEigen(wrench.torque);

  return addExternalStaticWrench(wrench_name, reference_frame, offset, wrench_eigen);
}

bool DragonRobotModel::removeExternalStaticWrench(const std::string wrench_name)
{
  if(external_wrench_map_.find(wrench_name) == external_wrench_map_.end())
    {
      ROS_WARN_STREAM("cannot find " << wrench_name << " to remove");
      return false;
    }

  external_wrench_map_.erase(wrench_name);

  return true;
}

void DragonRobotModel::resetExternalStaticWrench()
{
  external_wrench_map_.clear();
}


void DragonRobotModel::calcExternalWrenchCompThrust()
{
  const auto& seg_frames = getSegmentsTf();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const std::string baselink = getBaselinkName();
  const auto& thrust_wrench_allocations = getThrustWrenchAllocations();
  const auto& joint_positions = getJointPositions();
  const auto& thrust_coord_jacobians = getThrustCoordJacobians();

  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse());
  Eigen::VectorXd wrench_sum = Eigen::VectorXd::Zero(6);

  // get sum wrench from external wrench
  for(const auto& wrench : external_wrench_map_)
    {
      Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(6, 6);
      Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(wrench.second.frame) * wrench.second.offset);
      // p = root_rot * (aerial_robot_model::kdlToEigen(seg_frames.at(wrench.second.frame)) * wrench.second.offset);
      jacobi_root.topRightCorner(3,3) = - aerial_robot_model::skew(p);
      wrench_sum += jacobi_root.transpose() * wrench.second.wrench;
    }

  // TODO: redandunt!
  for (unsigned int i = 0; i < rotor_num; ++i)
    vectoring_q_mat_.middleCols(3 * i, 3) = thrust_wrench_allocations.at(i).leftCols(3);
  wrench_comp_thrust_ = aerial_robot_model::pseudoinverse(vectoring_q_mat_) * (-wrench_sum);

  ROS_DEBUG_STREAM("wrench_comp_thrust: " << wrench_comp_thrust_.transpose());

  // add external wrench compasation term
  Eigen::VectorXd static_thrust = getStaticThrust();
  // ROS_INFO_STREAM("only gravity static thrust: " << static_thrust.transpose());
  for(int i = 0; i < getRotorNum(); i++)
    {
      vectoring_thrust_(3 * i) = wrench_comp_thrust_(3 * i);
      vectoring_thrust_(3 * i + 1) = wrench_comp_thrust_(3 * i + 1);
      vectoring_thrust_(3 * i + 2) = wrench_comp_thrust_(3 * i + 2) + static_thrust(i);
      static_thrust(i) = vectoring_thrust_.segment(3 * i, 3).norm();
    }
  // ROS_INFO_STREAM("add wrench comp static thrust: " << static_thrust.transpose());

  setStaticThrust(static_thrust);

  // joint torque
  Eigen::VectorXd joint_torque = getJointTorque();
  // external wrench
  for(const auto& wrench : external_wrench_map_)
    {
      Eigen::MatrixXd coord_jacobian = RobotModel::getJacobian(joint_positions, wrench.second.frame, wrench.second.offset);
      joint_torque -= coord_jacobian.rightCols(joint_num).transpose() * wrench.second.wrench;
    }

  // wrench compensation thrust
  for (int i = 0; i < rotor_num; ++i) {
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench.head(3) = wrench_comp_thrust_.segment(3 * i, 3);
    joint_torque -= thrust_coord_jacobians.at(i).rightCols(joint_num).transpose() * wrench;
  }
  setJointTorque(joint_torque);
}

void DragonRobotModel::calcCompThrustJacobian()
{
  // w.r.t root
  const auto& joint_positions = getJointPositions();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = getThrustCoordJacobians().at(0).cols();
  Eigen::MatrixXd q_pseudo_inv = aerial_robot_model::pseudoinverse(vectoring_q_mat_);

  /* derivative for external wrench jacobian */
  Eigen::MatrixXd wrench_external_wrench_jacobian = Eigen::MatrixXd::Zero(6, ndof);
  for(const auto& wrench : external_wrench_map_)
    {
      auto f = wrench.second.wrench.head(3);
      wrench_external_wrench_jacobian.bottomRows(3) -= aerial_robot_model::skew(f) * getSecondDerivativeRoot(wrench.second.frame, wrench.second.offset);
    }
  comp_thrust_jacobian_ = -q_pseudo_inv * wrench_external_wrench_jacobian;

  ROS_DEBUG_STREAM("wrench_external_thrust_jacobian w.r.t. root : \n" << wrench_external_wrench_jacobian);

  /* derivative for thrust jacobian */
  std::vector<Eigen::MatrixXd> p_jacobians;
  Eigen::MatrixXd q_inv_jacobian = Eigen::MatrixXd::Zero(6, ndof);
  for (int i = 0; i < rotor_num; ++i)
    {
      std::string thrust_name = std::string("thrust") + std::to_string(i + 1);
      Eigen::MatrixXd p_jacobian = Eigen::MatrixXd::Zero(3, ndof);

      auto f = wrench_comp_thrust_.segment(3 * i, 3);
      p_jacobian = getSecondDerivativeRoot(thrust_name);
      q_inv_jacobian.bottomRows(3) += -aerial_robot_model::skew(f) * p_jacobian;
      p_jacobians.push_back(p_jacobian);
    }

  ROS_DEBUG_STREAM("wrench_external q_inv_jacobian: \n" << q_inv_jacobian);
  comp_thrust_jacobian_ += -q_pseudo_inv * q_inv_jacobian;

  Eigen::MatrixXd q_pseudo_inv_jacobian = Eigen::MatrixXd::Zero(3 * rotor_num, ndof);
  Eigen::VectorXd pseudo_wrech = q_pseudo_inv.transpose() * wrench_comp_thrust_;
  for(int i = 0; i < rotor_num; i++)
    q_pseudo_inv_jacobian.middleRows(3 * i, 3) = aerial_robot_model::skew(pseudo_wrech.tail(3)) * p_jacobians.at(i);

  comp_thrust_jacobian_ += (Eigen::MatrixXd::Identity(3 * rotor_num, 3 * rotor_num) - q_pseudo_inv * vectoring_q_mat_) * q_pseudo_inv_jacobian;

  ROS_DEBUG_STREAM("comp_thrust_jacobian: \n" << comp_thrust_jacobian_);
}


void DragonRobotModel::compThrustNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result)
{
  const std::map<std::string, KDL::Frame> seg_frames = getSegmentsTf();
  const int rotor_num = getRotorNum();
  const int full_body_dof = 6 + getLinkJointIndices().size();
  KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse();


  double delta_angle = 0.00001; // [rad]
  int col_index = 6;

  Eigen::VectorXd nominal_wrench_comp_thrust = wrench_comp_thrust_;
  Eigen::MatrixXd J_f = Eigen::MatrixXd::Zero(3 * getRotorNum(), full_body_dof);
  auto perturbation = [&](int col, KDL::JntArray joint_angles)
    {
      updateRobotModelImpl(joint_angles);
      calcExternalWrenchCompThrust();
      J_f.col(col) = (wrench_comp_thrust_ - nominal_wrench_comp_thrust) / delta_angle;
    };

  for (const auto& joint_index : getLinkJointIndices()) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    updateRobotModelImpl(perturbation_joint_positions);
    setCogDesireOrientation(root_rot * getSegmentsTf().at(getBaselinkName() ).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(getBaselinkName()).M);
  perturbation(3, joint_positions);

  // pitch
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(getBaselinkName()).M);
  perturbation(4, joint_positions);

  // yaw
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(getBaselinkName()).M);
  perturbation(5, joint_positions);

  // reset
  setCogDesireOrientation(baselink_rot); // set the orientation of root
  updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical result of wrench compensation thrust jacobian: \n" << J_f);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_DEBUG_STREAM("analytical_result: \n" << analytical_result);
      ROS_DEBUG_STREAM("diff of  jacobian: \n" << J_f - analytical_result);

      double min_diff = (J_f - analytical_result).minCoeff();
      double max_diff = (J_f - analytical_result).maxCoeff();

      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of wrench compensation thrust  jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of wrench compensation thrust  jacobian: " << fabs(min_diff));
    }
}

void DragonRobotModel::addCompThrustToLambdaJacobian()
{
  const Eigen::MatrixXd& lambda_jacobian = getLambdaJacobian();
  Eigen::MatrixXd augmented_lambda_jacobian = lambda_jacobian;
  for(int i = 0; i < getRotorNum(); i++)
    {
      Eigen::MatrixXd vector_f_jacobi = comp_thrust_jacobian_.middleRows(3 * i, 3);
      vector_f_jacobi.row(2) += lambda_jacobian.row(i);
      augmented_lambda_jacobian.row(i) = vectoring_thrust_.segment(3 * i, 3).transpose() * vector_f_jacobi / vectoring_thrust_.segment(3 * i, 3).norm();
    }
  setLambdaJacobian(augmented_lambda_jacobian);
}

void DragonRobotModel::addCompThrustToJointTorqueJacobian()
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = getLambdaJacobian().cols();
  const auto& thrust_coord_jacobians = getThrustCoordJacobians();

  Eigen::MatrixXd augmented_joint_torque_jacobian = Eigen::MatrixXd::Zero(joint_num, ndof);

  // external wrench
  for(const auto& wrench : external_wrench_map_)
    {
      for (int j = 0; j < joint_num; ++j)
        {
          augmented_joint_torque_jacobian.row(j) += wrench.second.wrench.transpose() * getSecondDerivative(wrench.second.frame, j, wrench.second.offset);
        }
    }

  // external wrench compensation thrust
  for (int i = 0; i < rotor_num; ++i)
    {
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      wrench.head(3) = wrench_comp_thrust_.segment(3 * i, 3);
      std::string thrust_name = std::string("thrust") + std::to_string(i + 1);

      for (int j = 0; j < joint_num; ++j)
        augmented_joint_torque_jacobian.row(j) += wrench.transpose() * getSecondDerivative(thrust_name, j);

      Eigen::MatrixXd thrust_wrench_jacobi = Eigen::MatrixXd::Zero(6, ndof);
      thrust_wrench_jacobi.topRows(3) = comp_thrust_jacobian_.middleRows(3 * i, 3);
      augmented_joint_torque_jacobian += thrust_coord_jacobians.at(i).rightCols(joint_num).transpose() * thrust_wrench_jacobi;

    }
  augmented_joint_torque_jacobian *= -1;

  setJointTorqueJacobian(augmented_joint_torque_jacobian + getJointTorqueJacobian());
}


void DragonRobotModel::thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
{
  const std::map<std::string, KDL::Frame> seg_frames = getSegmentsTf();
  if(joint_indices.empty()) joint_indices = getJointIndices();
  const int full_body_dof = 6 + joint_indices.size();
  std::string baselink = getBaselinkName();
  KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  double delta_angle = 0.00001; // [rad]

  Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(getRotorNum(), full_body_dof);

  calcExternalWrenchCompThrust();
  Eigen::VectorXd nominal_static_thrust = getStaticThrust();
  // ROS_INFO_STREAM("nominal_static_thrust: " << nominal_static_thrust.transpose());

  int col_index = 6;

  auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles){
    updateRobotModelImpl(joint_angles);
    calcExternalWrenchCompThrust();
    J_lambda.col(col) = (getStaticThrust() - nominal_static_thrust) / delta_angle;
  };

  col_index = 6;
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    updateRobotModelImpl(perturbation_joint_positions);
    setCogDesireOrientation(root_rot * getSegmentsTf().at(baselink).M); // necessary
    perturbationStaticThrust(col_index, perturbation_joint_positions);
    col_index++;
  }

  // virtual 6dof root
  // roll
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(baselink).M);
  perturbationStaticThrust(3, joint_positions);

  // pitch
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(baselink).M);
  perturbationStaticThrust(4, joint_positions);

  // yaw
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(baselink).M);
  perturbationStaticThrust(5, joint_positions);

  // reset
  setCogDesireOrientation(baselink_rot);
  updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical lambda_jacobian: \n" << J_lambda);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_DEBUG_STREAM("analytical lambda_jacobian: \n" << analytical_result);
      ROS_DEBUG_STREAM("diff of lambda jacobian: \n" << J_lambda - analytical_result);

      double min_diff = (J_lambda - analytical_result).minCoeff();
      double max_diff = (J_lambda - analytical_result).maxCoeff();
      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of lambda jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of lambda jacobian: " << fabs(min_diff));
    }
}

void DragonRobotModel::jointTorqueNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
{
  const auto& seg_frames = getSegmentsTf();
  if(joint_indices.empty()) joint_indices = getJointIndices();
  const int full_body_dof = 6 + joint_indices.size();
  Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(getJointNum(), full_body_dof);
  std::string baselink = getBaselinkName();
  KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
  calcJointTorque();
  calcExternalWrenchCompThrust();

  double delta_angle = 0.00001; // [rad]
  int col_index = 6;
  Eigen::VectorXd nominal_joint_torque = getJointTorque();

  auto perturbationJointTorque = [&](int col, KDL::JntArray joint_angles)
    {
      updateRobotModelImpl(joint_angles);
      calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
      calcJointTorque();
      calcExternalWrenchCompThrust();
      J_t.col(col) = (getJointTorque() - nominal_joint_torque) / delta_angle;
    };

  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    updateRobotModelImpl(perturbation_joint_positions);
    setCogDesireOrientation(root_rot * getSegmentsTf().at(baselink).M);
    perturbationJointTorque(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(baselink).M);
  perturbationJointTorque(3, joint_positions);

  // pitch
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(baselink).M);
  perturbationJointTorque(4, joint_positions);

  // yaw
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(baselink).M);
  perturbationJointTorque(5, joint_positions);

  // reset
  setCogDesireOrientation(baselink_rot); // set the orientation of root
  updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical result of joint_torque_jacobian: \n" << J_t);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_DEBUG_STREAM("analytical_result: \n" << analytical_result);
      ROS_DEBUG_STREAM("diff of joint torque jacobian: \n" << J_t - analytical_result);

      double min_diff = (J_t - analytical_result).minCoeff();
      double max_diff = (J_t - analytical_result).maxCoeff();

      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of torque jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of torque jacobian: " << fabs(min_diff));
    }
}
