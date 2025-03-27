#include <dragon/model/hydrus_like_robot_model.h>

using namespace Dragon;

HydrusLikeRobotModel::HydrusLikeRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double fc_rp_min_thre, double epsilon, double edf_radius, double edf_max_tilt) :
  HydrusRobotModel(init_with_rosparam, verbose, fc_t_min_thre, fc_rp_min_thre, epsilon, 3),
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
}

void HydrusLikeRobotModel::getParamFromRos()
{
  ros::NodeHandle nh;
  nh.param("edf_radius", edf_radius_, 0.035); //70mm EDF
  nh.param("edf_max_tilt", edf_max_tilt_, 0.26); //15 [deg]
}

bool HydrusLikeRobotModel::stabilityCheck(bool verbose)
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

bool HydrusLikeRobotModel::overlapCheck(bool verbose)
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

void HydrusLikeRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
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
  const auto seg_frames = getSegmentsTf();
  std::vector<KDL::Vector> f_edfs;
  for(const auto& name: edf_names_)
    f_edfs.push_back((getCog<KDL::Frame>().Inverse() * seg_frames.at(name)).p);

  edfs_origin_from_cog_ = f_edfs;
}

Eigen::MatrixXd HydrusLikeRobotModel::getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset)
{
  Eigen::MatrixXd jacobian = aerial_robot_model::transformable::RobotModel::getJacobian(joint_positions, segment_name, offset);
  return jacobian * gimbal_jacobian_;
}

void HydrusLikeRobotModel::calcBasicKinematicsJacobian()
{
  aerial_robot_model::transformable::RobotModel::calcBasicKinematicsJacobian();

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

      Eigen::MatrixXd gimbal_rot_jacobian = Eigen::MatrixXd::Zero(3, 2);
      gimbal_rot_jacobian.col(0) = thrust_coord_jacobians.at(i).block(3, gimbal_roll_index, 3, 1);
      gimbal_rot_jacobian.col(1) = thrust_coord_jacobians.at(i).block(3, gimbal_pitch_index, 3, 1);

      Eigen::MatrixXd gimbal_joint_jacobian = - (Eigen::MatrixXd::Identity(2,3) * gimbal_rot_jacobian).inverse() * Eigen::MatrixXd::Identity(2,3) * joint_rot_jacobian;

      gimbal_jacobian_.row(gimbal_roll_index) = gimbal_joint_jacobian.row(0);
      gimbal_jacobian_.row(gimbal_pitch_index) = gimbal_joint_jacobian.row(1);
    }
}

void HydrusLikeRobotModel::calcCoGMomentumJacobian()
{
  setCOGJacobian(Eigen::MatrixXd::Zero(3, 6 + getJointNum()));
  setLMomentumJacobian(Eigen::MatrixXd::Zero(3, 6 + getJointNum()));

  aerial_robot_model::transformable::RobotModel::calcCoGMomentumJacobian();
}

void HydrusLikeRobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model)
{

  if(update_model) updateRobotModel(joint_positions);

  calcCoGMomentumJacobian(); // should be processed first!

  calcBasicKinematicsJacobian(); // need cog_jacobian_

  calcLambdaJacobian();

  calcJointTorque(false);

  calcJointTorqueJacobian();

  // external wrench realted jacobians
  calcExternalWrenchCompThrust();
  calcCompThrustJacobian();
  addCompThrustToStaticThrust();
  addCompThrustToLambdaJacobian();
  addCompThrustToJointTorque();
  addCompThrustToJointTorqueJacobian();

  calcFeasibleControlRollPitchDistsJacobian();

  // convert to jacobian only for link joint
  std::vector<Eigen::MatrixXd> u_jacobians = getUJacobians();
  for(auto& jacobian: u_jacobians)
    {
      jacobian = jacobian * gimbal_jacobian_;
    }
  setUJacobians(u_jacobians);

  std::vector<Eigen::MatrixXd> p_jacobians = getPJacobians();
  for(auto& jacobian: p_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setPJacobians(p_jacobians);

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

  Eigen::MatrixXd fc_rp_dists_jacobian = getFeasibleControlRollPitchDistsJacobian();
  setFeasibleControlRollPitchDistsJacobian(fc_rp_dists_jacobian * gimbal_jacobian_);

  // update jacobian for rotor overlap
  calcRotorOverlapJacobian();
}

void HydrusLikeRobotModel::calcRotorOverlapJacobian()
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

  Eigen::MatrixXd p_i_jacobian = getJacobian(getJointPositions(), edf_names_.at(rotor_i_)).topRows(3);
  Eigen::MatrixXd p_j_jacobian = getJacobian(getJointPositions(), edf_names_.at(rotor_j_)).topRows(3);
  Eigen::MatrixXd p_v_jacobian = p_i_jacobian + (closest_p_j(2) - closest_p_i(2)) / u.at(rotor_i_/2)(2) * u_jacobians.at(rotor_i_/2) + u.at(rotor_i_/2) / u.at(rotor_i_/2)(2) * (p_j_jacobian.row(2) - p_i_jacobian.row(2)) - u.at(rotor_i_/2) / std::pow(u.at(rotor_i_/2)(2), 2) * (closest_p_j(2) - closest_p_i(2)) * u_jacobians.at(rotor_i_/2).row(2);
  rotor_overlap_jacobian_ = (closest_p_v - closest_p_j).transpose() * (p_v_jacobian - p_j_jacobian) / (closest_p_v - closest_p_j).norm();
  rotor_overlap_jacobian_ -= tan(edf_max_tilt_) * (p_i_jacobian.row(2) - p_j_jacobian.row(2));
}


std::vector<int> HydrusLikeRobotModel::getClosestRotorIndices()
{
  std::vector<int> rotor_indices;
  rotor_indices.push_back(rotor_i_);
  rotor_indices.push_back(rotor_j_);
  return rotor_indices;
}

bool HydrusLikeRobotModel::addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const KDL::Vector offset, const Eigen::VectorXd wrench)
{
  const auto seg_frames = getSegmentsTf();
  if(getTree().getSegment(reference_frame) == getTree().getSegments().end())
    {
      ROS_WARN_STREAM(reference_frame << " can not be found in segment map.");
      return false;
    }

  external_wrench_map_[wrench_name] =  ExternalWrench{reference_frame, offset, wrench};

  for(const auto wrench: external_wrench_map_)
    {
      ROS_DEBUG_STREAM(wrench.first << ", pos: " << aerial_robot_model::kdlToEigen(wrench.second.offset).transpose() << ", wrench: " << wrench.second.wrench.transpose());
    }

  return true;
}

bool HydrusLikeRobotModel::addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const geometry_msgs::Point offset, const geometry_msgs::Wrench wrench)
{
  KDL::Vector offset_kdl;
  tf::pointMsgToKDL(offset, offset_kdl);
  Eigen::Matrix<double, 6, 1> wrench_eigen;
  tf::wrenchMsgToEigen(wrench, wrench_eigen);

  return addExternalStaticWrench(wrench_name, reference_frame, offset_kdl, wrench_eigen);
}

bool HydrusLikeRobotModel::addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const KDL::Vector offset, const KDL::Wrench wrench)
{
  Eigen::VectorXd wrench_eigen(6);
  wrench_eigen.head(3) = aerial_robot_model::kdlToEigen(wrench.force);
  wrench_eigen.tail(3) = aerial_robot_model::kdlToEigen(wrench.torque);

  return addExternalStaticWrench(wrench_name, reference_frame, offset, wrench_eigen);
}

bool HydrusLikeRobotModel::removeExternalStaticWrench(const std::string wrench_name)
{
  if(external_wrench_map_.find(wrench_name) == external_wrench_map_.end())
    {
      ROS_WARN_STREAM("cannot find " << wrench_name << " to remove");
      return false;
    }

  external_wrench_map_.erase(wrench_name);

  return true;
}

void HydrusLikeRobotModel::resetExternalStaticWrench()
{
  external_wrench_map_.clear();
}


void HydrusLikeRobotModel::calcExternalWrenchCompThrust()
{
  calcExternalWrenchCompThrust(external_wrench_map_);
}

void HydrusLikeRobotModel::calcExternalWrenchCompThrust(const std::map<std::string, Dragon::ExternalWrench>& external_wrench_map)
{
  const auto seg_frames = getSegmentsTf();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const std::string baselink = getBaselinkName();
  const auto& thrust_wrench_allocations = getThrustWrenchAllocations();

  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse());
  Eigen::VectorXd wrench_sum = Eigen::VectorXd::Zero(6);

  // get sum wrench from external wrench
  for(const auto& wrench : external_wrench_map)
    {
      Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(6, 6);
      Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(wrench.second.frame) * wrench.second.offset);
      jacobi_root.topRightCorner(3,3) = - aerial_robot_model::skew(p);
      wrench_sum += jacobi_root.transpose() * wrench.second.wrench;
    }

  // TODO: redandunt!
  for (unsigned int i = 0; i < rotor_num; ++i)
    vectoring_q_mat_.middleCols(3 * i, 3) = thrust_wrench_allocations.at(i).leftCols(3);
  wrench_comp_thrust_ = aerial_robot_model::pseudoinverse(vectoring_q_mat_) * (-wrench_sum);

  ROS_DEBUG_STREAM("wrench_comp_thrust: " << wrench_comp_thrust_.transpose());
}

void HydrusLikeRobotModel::addCompThrustToStaticThrust()
{
  Eigen::VectorXd static_thrust = getStaticThrust();

  for(int i = 0; i < getRotorNum(); i++)
    {
      vectoring_thrust_(3 * i) = wrench_comp_thrust_(3 * i);
      vectoring_thrust_(3 * i + 1) = wrench_comp_thrust_(3 * i + 1);
      vectoring_thrust_(3 * i + 2) = wrench_comp_thrust_(3 * i + 2) + static_thrust(i);
      static_thrust(i) = vectoring_thrust_.segment(3 * i, 3).norm();
    }
  setStaticThrust(static_thrust);
}

void HydrusLikeRobotModel::addCompThrustToJointTorque()
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const auto& joint_positions = getJointPositions();
  const auto& thrust_coord_jacobians = getThrustCoordJacobians();

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

void HydrusLikeRobotModel::calcCompThrustJacobian()
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
  Eigen::VectorXd pseudo_wrench = q_pseudo_inv.transpose() * wrench_comp_thrust_;
  for(int i = 0; i < rotor_num; i++)
    q_pseudo_inv_jacobian.middleRows(3 * i, 3) = aerial_robot_model::skew(pseudo_wrench.tail(3)) * p_jacobians.at(i);

  comp_thrust_jacobian_ += (Eigen::MatrixXd::Identity(3 * rotor_num, 3 * rotor_num) - q_pseudo_inv * vectoring_q_mat_) * q_pseudo_inv_jacobian;

  ROS_DEBUG_STREAM("comp_thrust_jacobian: \n" << comp_thrust_jacobian_);
}


void HydrusLikeRobotModel::addCompThrustToLambdaJacobian()
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

void HydrusLikeRobotModel::addCompThrustToJointTorqueJacobian()
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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Dragon::HydrusLikeRobotModel, aerial_robot_model::RobotModel);
