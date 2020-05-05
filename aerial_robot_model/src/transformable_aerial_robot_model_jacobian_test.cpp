#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {

  void RobotModel::thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
  {
    const std::map<std::string, KDL::Frame> seg_frames = getSegmentsTf();
    const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
    if(joint_indices.empty()) joint_indices = getJointIndices();
    const auto& sigma = getRotorDirection();
    const double m_f_rate = getMFRate();
    const int full_body_dof = 6 + joint_indices.size();
    KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
    KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse();

    double delta_angle = 0.00001; // [rad]
    Eigen::MatrixXd J_g = Eigen::MatrixXd::Zero(6, full_body_dof);

    Eigen::MatrixXd J_thrust = Eigen::MatrixXd::Zero(q_mat_.rows(), full_body_dof);
    Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(getRotorNum(), full_body_dof);

    Eigen::VectorXd nominal_static_thrust = static_thrust_;
    Eigen::VectorXd nominal_wrench_g = getGravityWrenchOnRoot();
    Eigen::VectorXd nominal_wrench_thrust = q_mat_ * static_thrust_;

    int col_index = 6;
#if 1
    auto perturbationSeparateForce = [&](int col, KDL::JntArray joint_angles) {
      updateRobotModelImpl(joint_angles);
      Eigen::VectorXd perturbated_wrench_g = getGravityWrenchOnRoot();
      J_g.col(col) = (perturbated_wrench_g - nominal_wrench_g) / delta_angle;
      calcWrenchMatrixOnRoot();
      Eigen::VectorXd perturbated_wrench_thrust = q_mat_ * nominal_static_thrust;
      J_thrust.col(col) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_angle;
    };

    // joint part
    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions(joint_positions);
      perturbation_joint_positions(joint_index) += delta_angle;
      updateRobotModelImpl(perturbation_joint_positions);
      setCogDesireOrientation(root_rot * getSegmentsTf().at(baselink_).M);
      perturbationSeparateForce(col_index, perturbation_joint_positions);
      col_index++;
    }
    // virtual 6dof root

    // roll
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(baselink_).M);
    perturbationSeparateForce(3, joint_positions);

    // pitch
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(baselink_).M);
    perturbationSeparateForce(4, joint_positions);

    // yaw
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(baselink_).M);
    perturbationSeparateForce(5, joint_positions);

    // reset
    setCogDesireOrientation(baselink_rot);
    updateRobotModelImpl(joint_positions);

    ROS_DEBUG_STREAM("numerical result of wrench_gravity_jacobian: \n" << J_g);
    ROS_DEBUG_STREAM("numerical result of wrench_thrust_jacobian: \n" << J_thrust);
#endif
    auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles){
        updateRobotModelImpl(joint_angles);
        J_lambda.col(col) = (static_thrust_ - nominal_static_thrust) / delta_angle;
      };

    col_index = 6;
    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions = joint_positions;
      perturbation_joint_positions(joint_index) += delta_angle;
      updateRobotModelImpl(perturbation_joint_positions);
      setCogDesireOrientation(root_rot * getSegmentsTf().at(baselink_).M); // necessary
      perturbationStaticThrust(col_index, perturbation_joint_positions);
      col_index++;
    }

    // virtual 6dof root
    // roll
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(baselink_).M);
    perturbationStaticThrust(3, joint_positions);

    // pitch
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(baselink_).M);
    perturbationStaticThrust(4, joint_positions);

    // yaw
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(baselink_).M);
    perturbationStaticThrust(5, joint_positions);

    // reset
    setCogDesireOrientation(baselink_rot);
    updateRobotModelImpl(joint_positions);

    ROS_DEBUG_STREAM("numerical  lambda_jacobian: \n" << J_lambda);

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

  void RobotModel::jointTorqueNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
  {
    const auto& seg_frames = getSegmentsTf();
    if(joint_indices.empty()) joint_indices = getJointIndices();
    const int full_body_dof = 6 + joint_indices.size();
    Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(getJointNum(), full_body_dof);
    KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
    KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse();

    calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
    calcJointTorque();

    double delta_angle = 0.00001; // [rad]
    int col_index = 6;
    Eigen::VectorXd nominal_joint_torque = joint_torque_;

    auto perturbationJointTorque = [&](int col, KDL::JntArray joint_angles)
      {
        updateRobotModelImpl(joint_angles);
        calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
        calcJointTorque();
        J_t.col(col) = (joint_torque_ - nominal_joint_torque) / delta_angle;
      };

    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions = joint_positions;
      perturbation_joint_positions(joint_index) += delta_angle;
      updateRobotModelImpl(perturbation_joint_positions);
      setCogDesireOrientation(root_rot * getSegmentsTf().at(baselink_).M);
      perturbationJointTorque(col_index, perturbation_joint_positions);
      col_index++;
    }

    // roll
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(baselink_).M);
    perturbationJointTorque(3, joint_positions);

    // pitch
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(baselink_).M);
    perturbationJointTorque(4, joint_positions);

    // yaw
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(baselink_).M);
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

#if 0
    col_index = 6;
    delta_angle = 0.00001; // [rad]
    const auto& inertia_map = getInertiaMap();
    std::vector<Eigen::MatrixXd> nominal_thrust_coord_jacobians = thrust_coord_jacobians_;
    std::vector<Eigen::MatrixXd> nominal_cog_coord_jacobians = cog_coord_jacobians_;
    Eigen::MatrixXd J_t_j = Eigen::MatrixXd::Zero(joint_indices.size(), full_body_dof);

    auto perturbationJointTorqueSeparate = [&](int col, KDL::JntArray joint_angles)
      {
        updateRobotModelImpl(joint_angles);
        calcBasicKinematicsJacobian(); // necessary for thrust_coord_jacobias
        calcJointTorque();
#if 1
        for (int i = 0; i < getRotorNum(); ++i) {
          J_t_j.col(col)  -= (thrust_coord_jacobians_.at(i) - nominal_thrust_coord_jacobians.at(i)).rightCols(joint_indices.size()).transpose() * thrust_wrench_units_.at(i) * static_thrust_(i) / delta_angle;
        }
#else
        int seg_index = 0;
        for(const auto& inertia : inertia_map) {
          J_t_j.col(col)  -= (cog_coord_jacobians_.at(seg_index) - nominal_cog_coord_jacobians.at(seg_index)).rightCols(joint_indices.size()).transpose() * inertia.second.getMass() * (-gravity_)  / delta_angle;
          seg_index ++;
        }

#endif
      };

    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions = joint_positions;
      perturbation_joint_positions(joint_index) += delta_angle;
      updateRobotModelImpl(perturbation_joint_positions);
      setCogDesireOrientation(root_rot * getSegmentsTf().at(baselink_).M);
      perturbationJointTorqueSeparate(col_index, perturbation_joint_positions);
      col_index++;
    }

    // roll
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(baselink_).M);
    perturbationJointTorqueSeparate(3, joint_positions);

    // pitch
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(baselink_).M);
    perturbationJointTorqueSeparate(4, joint_positions);

    // yaw
    setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(baselink_).M);
    perturbationJointTorqueSeparate(5, joint_positions);

    // reset
    setCogDesireOrientation(root_rot); // set the orientation of root
    updateRobotModelImpl(joint_positions);

    Eigen::MatrixXd analytical_result_dash = Eigen::MatrixXd::Zero(joint_indices.size(), full_body_dof);
    int i = 0;
    for(int j = 0; j <  getJointIndices().size(); j++)
      {
        if(getJointIndices().at(j) == joint_indices.at(i))
          {
            analytical_result_dash.row(i) = analytical_result.row(j);
            i++;
          }
        if(i == joint_indices.size()) break;
      }

    ROS_INFO_STREAM("analytical_result of separate joint torque jacobian: \n" << analytical_result_dash);
    ROS_INFO_STREAM("numerical result of  separate joint torque jacobian: \n" << J_t_j);
    ROS_INFO_STREAM("diff: \n" << J_t_j - analytical_result_dash);
#endif
  }

  void RobotModel::cogMomentumNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_cog_result, Eigen::MatrixXd analytical_momentum_result, std::vector<int> joint_indices)
  {
    const auto& inertia_map = getInertiaMap();
    if(joint_indices.empty()) joint_indices = getJointIndices();
    const int full_body_dof = 6 + joint_indices.size();
    double mass_all = getMass();
    const std::map<std::string, KDL::Frame> nominal_seg_frames = getSegmentsTf();
    KDL::Rotation nominal_root_rot = getCogDesireOrientation<KDL::Rotation>() * nominal_seg_frames.at(baselink_).M.Inverse();

    Eigen::MatrixXd J_cog = Eigen::MatrixXd::Zero(3, full_body_dof);
    Eigen::MatrixXd J_L = Eigen::MatrixXd::Zero(3, full_body_dof);

    double delta_angle = 0.00001; // [rad]

    KDL::Vector nominal_cog = getCog<KDL::Frame>().p;

    auto perturbation = [&](int col, KDL::JntArray joint_angles, KDL::Rotation root_rot) {
      updateRobotModelImpl(joint_angles);
      const std::map<std::string, KDL::Frame> seg_frames = getSegmentsTf();

      for(const auto& seg : inertia_map)
        {
          Eigen::Vector3d p_momentum_jacobian = aerial_robot_model::kdlToEigen(root_rot * (seg_frames.at(seg.first) * seg.second.getCOG()) - nominal_root_rot * (nominal_seg_frames.at(seg.first)  * seg.second.getCOG())) * seg.second.getMass() / delta_angle;

          J_cog.col(col) += p_momentum_jacobian / mass_all;

          J_L.col(col) += aerial_robot_model::kdlToEigen(nominal_root_rot * (nominal_seg_frames.at(seg.first) * seg.second.getCOG() - nominal_cog)).cross(p_momentum_jacobian);

          KDL::Rotation inertia_rot = nominal_root_rot * nominal_seg_frames.at(seg.first).M;
          KDL::RigidBodyInertia seg_inertia = seg.second;
          Eigen::MatrixXd rotional_inertia = aerial_robot_model::kdlToEigen((inertia_rot * seg_inertia.RefPoint(seg.second.getCOG())).getRotationalInertia());

          KDL::Rotation pertuabated_inertia_rot = root_rot * seg_frames.at(seg.first).M;
          Eigen::MatrixXd omega_skew = (aerial_robot_model::kdlToEigen(pertuabated_inertia_rot) - aerial_robot_model::kdlToEigen(inertia_rot)) / delta_angle * aerial_robot_model::kdlToEigen(inertia_rot.Inverse());

          // ROS_INFO_STREAM("omega_skew of " << seg.first << " for col " << col << ": \n" << omega_skew);
          Eigen::Vector3d omega(omega_skew(2,1), omega_skew(0,2), omega_skew(1,0));
          J_L.col(col) += rotional_inertia * omega;
        }

      // simple way to get cog velocity jacobian
      //J_cog.col(col) = aerial_robot_model::kdlToEigen(root_rot * getCog<KDL::Frame>().p - nominal_root_rot * nominal_cog) / delta_angle;
    };

    /* joint */
    int col_index = 6;
    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions(joint_positions);
      perturbation_joint_positions(joint_index) += delta_angle;
      perturbation(col_index, perturbation_joint_positions, nominal_root_rot);
      col_index++;
    }

    // virtual 6dof root
    J_cog.leftCols(3) =  aerial_robot_model::kdlToEigen(nominal_root_rot);
    // roll
    perturbation(3, joint_positions, nominal_root_rot * KDL::Rotation::RPY(delta_angle, 0, 0));

    // pitch
    perturbation(4, joint_positions, nominal_root_rot * KDL::Rotation::RPY(0, delta_angle, 0));

    // yaw
    perturbation(5, joint_positions, nominal_root_rot * KDL::Rotation::RPY(0, 0, delta_angle));

    // reset
    updateRobotModelImpl(joint_positions);

    ROS_DEBUG_STREAM("numerical cog_jacobian: \n" << J_cog);

    if(analytical_cog_result.cols() > 0 && analytical_cog_result.rows() > 0)
      {
        ROS_DEBUG_STREAM("analytical cog_jacobian: \n" << analytical_cog_result);
        ROS_DEBUG_STREAM("diff of cog jacobian: \n" << J_cog - analytical_cog_result);

        double min_diff = (J_cog - analytical_cog_result).minCoeff();
        double max_diff = (J_cog - analytical_cog_result).maxCoeff();
        if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of cog jacobian: " << max_diff);
        else  ROS_INFO_STREAM("max diff of cog jacobian: " << fabs(min_diff));
      }

    ROS_DEBUG_STREAM("numerical angular momentum_jacobian: \n" << J_L);

    if(analytical_momentum_result.cols() > 0 && analytical_momentum_result.rows() > 0)
      {
        ROS_DEBUG_STREAM("analytical angular momentum jacobian: \n" << analytical_momentum_result);
        ROS_DEBUG_STREAM("diff of two jacobians: \n" << J_L - analytical_momentum_result);

        double min_diff = (J_L - analytical_momentum_result).minCoeff();
        double max_diff = (J_L - analytical_momentum_result).maxCoeff();
        if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of angular momentum jacobian: " << max_diff);
        else  ROS_INFO_STREAM("max diff of angular momentum jacobian: " << fabs(min_diff));
      }
  }
};
