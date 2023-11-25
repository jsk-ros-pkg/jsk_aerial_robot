#include <dragon/numerical_jacobians.h>

DragonNumericalJacobian::DragonNumericalJacobian(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<aerial_robot_model::transformable::RobotModel> robot_model):
  HydrusNumericalJacobian(nh, nhp, std::move(robot_model))
{
  nhp_.param("check_rotor_overlap", check_rotor_overlap_, true);
  nhp_.param("check_comp_thrust", check_comp_thrust_, true);

  nhp_.param("comp_thrust_diff_thre", comp_thrust_diff_thre_, 0.001);
  nhp_.param("rotor_overlap_diff_thre", rotor_overlap_diff_thre_, 0.001);

  Eigen::Vector3d f;
  nhp_.param("external_fx", f.x(), 0.0);
  nhp_.param("external_fy", f.y(), 0.0);
  nhp_.param("external_fz", f.z(), 0.0);
  std::string external_f_frame;
  nhp_.param("external_f_frame", external_f_frame, std::string("link1"));
  double external_f_offset;
  nhp_.param("external_f_offset", external_f_offset, 0.0);

  if(f != Eigen::Vector3d::Zero() && check_comp_thrust_)
    {
      ROS_INFO_STREAM("Add external force " << f.transpose() << " to " << external_f_frame << " with offset " << external_f_offset);
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      wrench.head(3) = f;
      getDragonRobotModel().addExternalStaticWrench("force1", "link4", KDL::Vector(external_f_offset, 0, 0), wrench);
    }
}


bool DragonNumericalJacobian::checkJacobians()
{
  bool flag = true;
  const auto& link_joint_indices = getRobotModel().getLinkJointIndices();
  if(check_comp_thrust_)  flag &= checkExternalWrenchCompensateThrustJacobian();
  if(check_thrust_force_) flag &= checkThrsutForceJacobian(link_joint_indices);
  if(check_joint_torque_) flag &= checkJointTorqueJacobian(link_joint_indices);
  if(check_cog_motion_) flag &= checkCoGMomentumJacobian(link_joint_indices);
  if(check_rotor_overlap_) flag &= checkRotorOverlapJacobian();
  if(check_feasible_control_roll_pitch_) flag &= checkFeasibleControlRollPitchJacobian(link_joint_indices);

  return flag;
}

const Eigen::MatrixXd DragonNumericalJacobian::thrustForceNumericalJacobian(std::vector<int> joint_indices)
{
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const std::string baselink = getRobotModel().getBaselinkName();
  const int rotor_num = getRobotModel().getRotorNum();
  const int full_body_dof = 6 + joint_indices.size();

  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(rotor_num, full_body_dof);

  getDragonRobotModel().calcExternalWrenchCompThrust();
  getDragonRobotModel().addCompThrustToStaticThrust();
  const Eigen::VectorXd nominal_static_thrust = getRobotModel().getStaticThrust();

  int col_index = 6;

  auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles){
    getRobotModel().updateRobotModel(joint_angles);
    getDragonRobotModel().calcExternalWrenchCompThrust();
    getDragonRobotModel().addCompThrustToStaticThrust();
    const Eigen::VectorXd static_thrust = getRobotModel().getStaticThrust();
    J_lambda.col(col) = (static_thrust - nominal_static_thrust) / delta_;
  };

  col_index = 6;
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M); // necessary
    perturbationStaticThrust(col_index, perturbation_joint_positions);
    col_index++;
  }

  // virtual 6dof root
  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbationStaticThrust(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbationStaticThrust(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbationStaticThrust(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot);
  getRobotModel().updateRobotModel(joint_positions);

  ROS_DEBUG_STREAM("numerical lambda_jacobian: \n" << J_lambda);

  return J_lambda;
}

bool DragonNumericalJacobian::checkThrsutForceJacobian(std::vector<int> joint_indices)
{
  return aerial_robot_model::NumericalJacobian::checkThrsutForceJacobian(joint_indices);
}

const Eigen::MatrixXd DragonNumericalJacobian::jointTorqueNumericalJacobian(std::vector<int> joint_indices)
{
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int full_body_dof = 6 + joint_indices.size();
  Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(getRobotModel().getJointNum(), full_body_dof);
  const std::string baselink = getRobotModel().getBaselinkName();
  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  getDragonRobotModel().calcExternalWrenchCompThrust();
  getRobotModel().calcJointTorque();
  getDragonRobotModel().addCompThrustToJointTorque();

  int col_index = 6;
  const Eigen::VectorXd nominal_joint_torque = getRobotModel().getJointTorque();

  auto perturbationJointTorque = [&](int col, KDL::JntArray joint_angles)
    {
      getRobotModel().updateRobotModel(joint_angles);
      getDragonRobotModel().calcExternalWrenchCompThrust();
      getRobotModel().calcJointTorque();
      getDragonRobotModel().addCompThrustToJointTorque();
      const Eigen::VectorXd joint_torque = getRobotModel().getJointTorque();
      J_t.col(col) = (joint_torque - nominal_joint_torque) / delta_;
    };

  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
    perturbationJointTorque(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbationJointTorque(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbationJointTorque(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbationJointTorque(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot); // set the orientation of root
  getRobotModel().updateRobotModel(joint_positions);

  ROS_DEBUG_STREAM("numerical result of joint_torque_jacobian: \n" << J_t);

  return J_t;
}

const Eigen::MatrixXd DragonNumericalJacobian::overlapNumericalJacobian()
{
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int rotor_num = getRobotModel().getRotorNum();
  const std::string baselink = getRobotModel().getBaselinkName();
  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();
  const double edf_max_tilt = getDragonRobotModel().getEdfMaxTilt();
  const std::vector<KDL::Vector> nominal_edfs_origin_from_cog = getDragonRobotModel().getEdfsOriginFromCog<KDL::Vector>();
  const auto closest_rotor_indices = getDragonRobotModel().getClosestRotorIndices();

  Eigen::MatrixXd J_overlap = Eigen::MatrixXd::Zero(1, 6 + getRobotModel().getLinkJointIndices().size());

  int col_index = 6;



  auto perturbation = [&](int col, KDL::JntArray joint_angles)
    {
      getDragonRobotModel().updateRobotModel(joint_angles);
      const std::vector<KDL::Vector> edfs_origin_from_cog = getDragonRobotModel().getEdfsOriginFromCog<KDL::Vector>();
      const double min_dist =   getDragonRobotModel().getClosestRotorDist();
      auto diff = edfs_origin_from_cog.at(closest_rotor_indices.at(0)) - edfs_origin_from_cog.at(closest_rotor_indices.at(1));
      double d_tilt = diff.z() * tan(edf_max_tilt);
      diff.z(0);
      J_overlap(0, col) = (diff.Norm() - d_tilt - min_dist) / delta_;
    };

  for (const auto& joint_index : getRobotModel().getLinkJointIndices()) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbation(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbation(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbation(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot);
  getRobotModel().updateRobotModel(joint_positions);

  ROS_DEBUG_STREAM("numerical result of rotor overlap jacobian: \n" << J_overlap);

  return J_overlap;
}

bool DragonNumericalJacobian::checkRotorOverlapJacobian()
{
  const Eigen::MatrixXd analytical_jacobi = getDragonRobotModel().getRotorOverlapJacobian();
  const Eigen::MatrixXd numerical_jacobi = overlapNumericalJacobian();
  ROS_DEBUG_STREAM("analytical rotor overlap jacobian: \n" << analytical_jacobi);
  ROS_DEBUG_STREAM("diff of rotor overlap jacobian: \n" << numerical_jacobi - analytical_jacobi);

  double max_diff = (numerical_jacobi - analytical_jacobi).maxCoeff();
  double min_diff = (numerical_jacobi - analytical_jacobi).minCoeff();
  if(fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if(max_diff < rotor_overlap_diff_thre_)
    {
      ROS_INFO_STREAM("max diff of rotor overlap distance jacobian: " << max_diff);
      return true;
    }
  else
    {
      ROS_WARN_STREAM("max diff of rotor overlap distance jacobian: " << max_diff << ", exceed!");
      return false;
    }
}

const Eigen::MatrixXd DragonNumericalJacobian::compThrustNumericalJacobian()
{
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int rotor_num = getRobotModel().getRotorNum();
  const int full_body_dof = 6 + getRobotModel().getLinkJointIndices().size();
  const std::string baselink = getRobotModel().getBaselinkName();
  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  int col_index = 6;

  const Eigen::VectorXd nominal_wrench_comp_thrust = getDragonRobotModel().getExWrenchCompensateVectoringThrust();
  Eigen::MatrixXd J_f = Eigen::MatrixXd::Zero(3 * rotor_num, full_body_dof);
  auto perturbation = [&](int col, KDL::JntArray joint_angles)
    {
      getRobotModel().updateRobotModel(joint_angles);
      getDragonRobotModel().calcExternalWrenchCompThrust();
      const Eigen::VectorXd wrench_comp_thrust = getDragonRobotModel().getExWrenchCompensateVectoringThrust();
      J_f.col(col) = (wrench_comp_thrust - nominal_wrench_comp_thrust) / delta_;
    };

  for (const auto& joint_index : getRobotModel().getLinkJointIndices()) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbation(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbation(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbation(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot);
  getRobotModel().updateRobotModel(joint_positions);

  ROS_DEBUG_STREAM("numerical result of wrench compensation thrust jacobian: \n" << J_f);

  return J_f;
}

bool DragonNumericalJacobian::checkExternalWrenchCompensateThrustJacobian()
{
  const Eigen::MatrixXd analytical_jacobi = getDragonRobotModel().getCompThrustJacobian();
  const Eigen::MatrixXd numerical_jacobi = compThrustNumericalJacobian();
  ROS_DEBUG_STREAM("analytical external wrench comp thrust jacobian: \n" << analytical_jacobi);
  ROS_DEBUG_STREAM("diff of external wrench comp thrust jacobian: \n" << numerical_jacobi - analytical_jacobi);

  double max_diff = (numerical_jacobi - analytical_jacobi).maxCoeff();
  double min_diff = (numerical_jacobi - analytical_jacobi).minCoeff();
  if(fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if(max_diff < comp_thrust_diff_thre_)
    {
      ROS_INFO_STREAM("max diff of external wrench comp thrust jacobian: " << max_diff);
      return true;
    }
  else
    {
      ROS_WARN_STREAM("max diff of external wrench comp thrust jacobian: " << max_diff << ", exceed!");
      return false;
    }
}
