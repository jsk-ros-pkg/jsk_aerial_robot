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
          Eigen::Vector3d diff = edfs_origin_from_cog[i] - edfs_origin_from_cog[j]; //dual
          double projected_dist = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
          //approximated, the true one should be (edf_radius_ / cos(tilt) + diff(2) * tan(tilt)
          double dist_thre = edf_radius_ + fabs(diff(2)) * tan(edf_max_tilt_) + edf_radius_;
          /* special for dual rotor */
          if(i / 2 == j / 2) continue;

          /* debug */
          if(dist_thre > projected_dist)
            {
              if(verbose) ROS_WARN("overlap!: %d and %d, projectd_dist: %f, thre: %f",
                                   i + 1, j + 1, projected_dist, dist_thre);
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
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      std::string edf = std::string("edf") + s + std::string("_left");
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * seg_frames.at(edf)).p);
      edf = std::string("edf") + s + std::string("_right");
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * seg_frames.at(edf)).p);
    }
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

  // test
  // thrust_coord_jacobians = getThrustCoordJacobians();
  // for(auto& jacobian: thrust_coord_jacobians) ROS_INFO_STREAM("\n" << jacobian);
   // u_jacobians = getUJacobians();
   // for(auto& jacobian: u_jacobians)  ROS_INFO_STREAM("u jacobian: \n" << jacobian);

  thrustForceNumericalJacobian(getJointPositions(), getLambdaJacobian(), getLinkJointIndices());

  jointTorqueNumericalJacobian(getJointPositions(), getJointTorqueJacobian(), getLinkJointIndices());
  cogMomentumNumericalJacobian(getJointPositions(), getCOGJacobian(), getLMomentumJacobian(), getLinkJointIndices());

  //for(auto name: getLinkJointNames()) ROS_WARN_STREAM(name);
  //throw std::runtime_error("test");
}


void DragonRobotModel::thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
{
  aerial_robot_model::RobotModel::thrustForceNumericalJacobian(joint_positions, analytical_result, joint_indices);
}
