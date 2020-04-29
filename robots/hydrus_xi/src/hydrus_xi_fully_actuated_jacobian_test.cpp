#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>

void HydrusXiFullyActuatedRobotModel::thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result)
{
  const auto& seg_frames = getSegmentsTf();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& joint_indices = getJointIndices();
  const auto& sigma = getRotorDirection();
  const double m_f_rate = getMFRate();
  const int full_body_ndof = 6 + getJointNum();

  double delta_angle = 0.00001; // [rad]
  Eigen::MatrixXd J_g = Eigen::MatrixXd::Zero(6, full_body_ndof);

  Eigen::MatrixXd J_thrust = Eigen::MatrixXd::Zero(6, full_body_ndof);
  Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(getRotorNum(), full_body_ndof);

  KDL::Rotation curr_root_att = getCogDesireOrientation<KDL::Rotation>();

  calcStaticThrust();

  Eigen::VectorXd nominal_static_thrust = static_thrust_;
  Eigen::VectorXd nominal_wrench_g = getGravityWrenchOnRoot();
  Eigen::VectorXd nominal_wrench_thrust = q_mat_ * static_thrust_;

  int col_index = 6;

  auto perturbationSeparateForce = [&](int col, KDL::JntArray joint_angles) {
    aerial_robot_model::RobotModel::updateRobotModelImpl(joint_angles);
    Eigen::VectorXd perturbated_wrench_g = getGravityWrenchOnRoot();
    J_g.col(col) = (perturbated_wrench_g - nominal_wrench_g) / delta_angle;
    calcQMatrixOnRoot();
    Eigen::VectorXd perturbated_wrench_thrust = q_mat_ * nominal_static_thrust;
    J_thrust.col(col) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_angle;
  };

  // joint part
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray pertubation_joint_positions(joint_positions);
    pertubation_joint_positions(joint_index) += delta_angle;
    perturbationSeparateForce(col_index, pertubation_joint_positions);
    col_index++;
  }

  // virtual 6dof root

  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
  perturbationSeparateForce(3, joint_positions);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0));
  perturbationSeparateForce(4, joint_positions);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle));
  perturbationSeparateForce(5, joint_positions);

  // reset
  setCogDesireOrientation(curr_root_att);
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical result of wrench_gravity_jacobian: \n" << J_g);
  ROS_DEBUG_STREAM("numerical result of wrench_thrust_jacobian: \n" << J_thrust);

  auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles)
    {
      aerial_robot_model::RobotModel::updateRobotModelImpl(joint_angles);
      calcStaticThrust();
      J_lambda.col(col) = (static_thrust_ - nominal_static_thrust) / delta_angle;
    };

  col_index = 6;
  for (const auto& joint_index : joint_indices) {
      KDL::JntArray pertubation_joint_positions = joint_positions;
      pertubation_joint_positions(joint_index) += delta_angle;
      perturbationStaticThrust(col_index, pertubation_joint_positions);
      col_index++;
    }

  // virtual 6dof root
  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
  perturbationStaticThrust(3, joint_positions);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0));
  perturbationStaticThrust(4, joint_positions);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle));
  perturbationStaticThrust(5, joint_positions);

  // reset
  setCogDesireOrientation(curr_root_att); // set the orientation of root
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical  lambda_jacobian: \n" << J_lambda);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_DEBUG_STREAM("analytical lambda_jacobian: \n" << analytical_result);
      ROS_DEBUG_STREAM("diff of lambda jacobian: \n" << J_lambda - analytical_result);

      double min_diff = (J_lambda - analytical_result).minCoeff();
      double max_diff = (J_lambda - analytical_result).maxCoeff();
      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of labda jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of labda jacobian: " << fabs(min_diff));
    }
}

void HydrusXiFullyActuatedRobotModel::jointTorqueNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result)
{
  const auto& joint_indices = getJointIndices();
  const int full_body_ndof = 6 + getJointNum();
  Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(getJointNum(), full_body_ndof);

  KDL::Rotation curr_root_att = getCogDesireOrientation<KDL::Rotation>();

  calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
  calcStaticThrust();
  calcJointTorque();

  double delta_angle = 0.00001; // [rad]
  int col_index = 6;
  Eigen::VectorXd nominal_joint_torque = joint_torque_;

  auto perturbationJointTorque = [&](int col, KDL::JntArray joint_angles)
    {
      aerial_robot_model::RobotModel::updateRobotModelImpl(joint_angles);
      calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
      calcStaticThrust();
      calcJointTorque();
      J_t.col(col) = (joint_torque_ - nominal_joint_torque) / delta_angle;
    };

  for (const auto& joint_index : joint_indices) {
      KDL::JntArray pertubation_joint_positions = joint_positions;
      pertubation_joint_positions(joint_index) += delta_angle;
      perturbationJointTorque(col_index, pertubation_joint_positions);
      col_index++;
    }

  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0)); // set the orientation of root
  perturbationJointTorque(3, joint_positions);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0)); // set the orientation of root
  perturbationJointTorque(4, joint_positions);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle)); // set the orientation of root
  perturbationJointTorque(5, joint_positions);

  // reset
  setCogDesireOrientation(curr_root_att); // set the orientation of root
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical result of joint_torque_jacobian: \n" << J_t);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_DEBUG_STREAM("analytical_result: \n" << analytical_result);
      ROS_DEBUG_STREAM("diff of joint torque jacobian: \n" << J_t - analytical_result);

      if((J_t - analytical_result).maxCoeff() > fabs((J_t - analytical_result).minCoeff()))
        ROS_INFO_STREAM("max diff of torque jacobian: " << (J_t - analytical_result).maxCoeff());
      else
        ROS_INFO_STREAM("max diff of torque jacobian: " << fabs((J_t - analytical_result).minCoeff()));
    }

#if 0
  col_index = 6;
  delta_angle = 0.00001; // [rad]
  std::vector<Eigen::VectorXd> thrust_wrench = thrust_wrench_; // TODO
  std::vector<Eigen::MatrixXd> thrust_coord_jacobians = thrust_coord_jacobians_;
  std::vector<Eigen::MatrixXd> cog_coord_jacobians = cog_coord_jacobians_;
  Eigen::MatrixXd J_t_j = Eigen::MatrixXd::Zero(getJointNum(), full_body_ndof);

  auto virtualRootRotJacobianTorque = [&](int col, KDL::JntArray joint_angles)
    {
      aerial_robot_model::RobotModel::updateRobotModelImpl(joint_angles);
      calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
      calcStaticThrust();
      calcJointTorque();
#if 0
      for (int i = 0; i < getRotorNum(); ++i) {
        J_t_j.col(col)  -= (thrust_coord_jacobians_.at(i) - thrust_coord_jacobians.at(i)).rightCols(getJointNum()).transpose() * thrust_wrench.at(i)  / delta_angle;
      }
#else
      int seg_index = 0;
      for(const auto& inertia : inertia_map) {
        J_t_j.col(col)  -= (cog_coord_jacobians_.at(seg_index) - cog_coord_jacobians.at(seg_index)).rightCols(getJointNum()).transpose() * inertia.second.getMass() * (-gravity_)  / delta_angle;
        seg_index ++;
      }

#endif
    };

  for (const auto& joint_index : joint_indices) {
      KDL::JntArray pertubation_joint_positions = joint_positions;
      pertubation_joint_positions(joint_index) += delta_angle;
      aerial_robot_model::RobotModel::updateRobotModelImpl(pertubation_joint_positions);
      calcJointTorque();
#if 0
      for (int i = 0; i < getRotorNum(); ++i) {
        J_t_j.col(col_index)  -= (thrust_coord_jacobians_.at(i) - thrust_coord_jacobians.at(i)).rightCols(getJointNum()).transpose() * thrust_wrench.at(i)  / delta_angle;
      }
#else
      int seg_index = 0;
      for(const auto& inertia : inertia_map) {
        J_t_j.col(col_index)  -= (cog_coord_jacobians_.at(seg_index) - cog_coord_jacobians.at(seg_index)).rightCols(getJointNum()).transpose() * inertia.second.getMass() * (-gravity_)  / delta_angle;
        seg_index ++;
      }
#endif
      col_index++;
    }

  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0)); // set the orientation of root
  virtualRootRotJacobianTorque(3);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0)); // set the orientation of root
  virtualRootRotJacobianTorque(4);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle)); // set the orientation of root
  virtualRootRotJacobianTorque(5);

  // reset
  setCogDesireOrientation(curr_root_att); // set the orientation of root
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  ROS_INFO_STREAM("joint_torque_jacobian : \n" << joint_torque_jacobian);
  ROS_INFO_STREAM("numerical result of joint_torque_jacobian : \n" << J_t_j);
  ROS_INFO_STREAM("diff: \n" << J_t_j - joint_torque_jacobian);
#endif
}
