#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

// void RollingController::standingInitialize()
// {
// }


void RollingController::standingPlanning()
{
  /* set target roll of baselink */
  tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);

  double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
  double baselink_pitch = estimator_->getEuler(Frame::BASELINK, estimate_mode_).y();

  // if(std::abs(cog_pos.z() / circle_radius_) < 1.0 && std::asin(cog_pos.z() / circle_radius_) > standing_target_phi_)
  //   {
  //     if(baselink_roll < M_PI / 2.0)
  //       {
  //         standing_target_phi_ = std::asin(cog_pos.z() / circle_radius_) + 0.1;
  //       }
  //     else
  //       {
  //         standing_target_phi_ = std::asin(cog_pos.z() / circle_radius_) - 0.1;
  //       }
  //   }


  // spinal::DesireCoord desire_coordinate_msg;
  // desire_coordinate_msg.roll = standing_target_phi_;
  // desire_coordinate_msg.pitch = baselink_pitch;
  // desire_coordinate_pub_.publish(desire_coordinate_msg);

  pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

}

void RollingController::calcStandingFullLambda()
{
  int n_variables = 2 * motor_num_;
  int n_constraints = 3 + 1 + 2 + 2;
  double epsilon = 0.0001;
  double large_epsilon = 0.1;

  Eigen::MatrixXd H(n_variables, n_variables);
  H <<
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::SparseMatrix<double> H_s;
  H_s = H.sparseView();

  /* normal pid result */
  Eigen::VectorXd target_wrench_acc_target_frame;
  target_wrench_acc_target_frame.resize(6);
  target_wrench_acc_target_frame.tail(3) = Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                           pid_controllers_.at(PITCH).result(),
                                                           pid_controllers_.at(YAW).result());

  // std::cout << "pid result = [" << target_wrench_acc_target_frame(ROLL) << " " << target_wrench_acc_target_frame(PITCH) << " " << target_wrench_acc_target_frame(YAW) << "] " << std::endl;

  /* consider gravity force as feed-forward compensation
     this torque is described based on desired baselink angle */
  Eigen::Vector3d gravity_torque_from_target_frame = rolling_robot_model_->getGravityTorqueFromTargetFrame();
  Eigen::Vector3d gravity_ang_acc_from_target_frame = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse() * gravity_torque_from_target_frame;

  // std::cout << "gravity torque from target frame [" << gravity_torque_from_target_frame(0) << " " << gravity_torque_from_target_frame(1) << " " << gravity_torque_from_target_frame(2) << "] " << std::endl;
  // std::cout << "gravity ang acc from target frame [" << gravity_ang_acc_from_target_frame(0) << " " << gravity_ang_acc_from_target_frame(1) << " " << gravity_ang_acc_from_target_frame(2) << "] " << std::endl;

  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = robot_model_->getMass() * contact_point_alined_to_cog_p_skew * gravity;

  // std::cout << "gravity ang acc from contact point alined = [" << gravity_ang_acc_from_contact_point_alined(0) << " " << gravity_ang_acc_from_contact_point_alined(1) << " " << gravity_ang_acc_from_contact_point_alined(2) << " " << std::endl;

  // tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  // tf::Matrix3x3 uav_rp_rot = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z())).inverse() * uav_rot;
  // tf::Matrix3x3 uav_yaw_rot = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()));
  // Eigen::Matrix3d uav_rot_eigen;
  // Eigen::Matrix3d uav_rp_rot_eigen;
  // Eigen::Matrix3d uav_yaw_rot_eigen;
  // matrixTFToEigen(uav_rot, uav_rot_eigen);
  // matrixTFToEigen(uav_rp_rot, uav_rp_rot_eigen);
  // matrixTFToEigen(uav_yaw_rot, uav_yaw_rot_eigen);
  // std::cout << "uav_rot = \n" << uav_rot_eigen << std::endl;
  // std::cout << "uav_rp_rot = \n" << uav_rp_rot_eigen << std::endl;
  // std::cout << "uav_yaw_rot = \n" << uav_yaw_rot_eigen << std::endl;

  /* use sum of pid result and gravity compensation torque for attitude control */
  target_wrench_acc_target_frame.tail(3) = target_wrench_acc_target_frame.tail(3) + gravity_ang_acc_from_contact_point_alined;

  // std::cout << "target ang acc in target frame [" << target_wrench_acc_target_frame(ROLL) << " " << target_wrench_acc_target_frame(PITCH) << " " << target_wrench_acc_target_frame(YAW) << " " << std::endl;

  // Eigen::VectorXd full_lambda_rot = aerial_robot_model::pseudoinverse(full_q_mat_target_frame_.bottomRows(3)) * target_wrench_acc_cog.tail(3);
  // std::cout << "calced full_lambda_rot" << std::endl;

  Eigen::MatrixXd full_q_mat_trans = robot_model_->getMass() * full_q_trans_target_frame_;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.topRows(3) = full_q_rot_target_frame_;                                                           //    eq constraint about rpy torque
  A.block(3, 0, 1, n_variables) = full_q_mat_trans.row(Z);                                           // in eq constraint about z
  A.block(4, 0, 1, n_variables) = full_q_mat_trans.row(X) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(5, 0, 1, n_variables) = full_q_mat_trans.row(X) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(6, 0, 1, n_variables) = full_q_mat_trans.row(Y) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y
  A.block(7, 0, 1, n_variables) = full_q_mat_trans.row(Y) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y

  // std::cout << "set A" << std::endl;
  // std::cout << A << std::endl;

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  // std::cout << lower_bound << std::endl;

  lower_bound
    <<
    target_wrench_acc_target_frame(ROLL) - epsilon,
    target_wrench_acc_target_frame(PITCH) - epsilon,
    target_wrench_acc_target_frame(YAW) - large_epsilon,
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY;

  // std::cout << lower_bound << std::endl;

  upper_bound <<
    target_wrench_acc_target_frame(ROLL) + epsilon,
    target_wrench_acc_target_frame(PITCH) + epsilon,
    target_wrench_acc_target_frame(YAW) + large_epsilon,
    robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);

  // std::cout << "set boundarys" << std::endl;

  OsqpEigen::Solver solver;

  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(n_variables);
  solver.data()->setNumberOfConstraints(n_constraints);
  solver.data()->setHessianMatrix(H_s);
  solver.data()->setLinearConstraintsMatrix(A_s);
  solver.data()->setGradient(gradient);
  solver.data()->setLowerBound(lower_bound);
  solver.data()->setUpperBound(upper_bound);

  if(!solver.initSolver())
    {
      std::cout << "init solver error" << std::endl;
    }
  solver.solve();

  auto solution = solver.getSolution();

  full_lambda_all_ = solution;
  full_lambda_trans_ = solution;

  // std::cout << "exerted wrench in target frame = [";
  Eigen::VectorXd exerted_wrench_target_frame = full_q_mat_target_frame_ * solution;
  // for(int i = 0; i < exerted_wrench_target_frame.size(); i++)
  //   {
  //     std::cout << exerted_wrench_target_frame(i) << " ";
  //   }
  // std::cout << std::endl;

  /* confirm wrench on cog is correct */
  KDL::Frame cog_to_target_frame = rolling_robot_model_->getCogToTargetFrame<KDL::Frame>();
  Eigen::Vector3d cog_to_target_frame_p = Eigen::Vector3d(cog_to_target_frame.p.x(),
                                                          cog_to_target_frame.p.y(),
                                                          cog_to_target_frame.p.z());
  Eigen::Matrix3d cog_to_target_frame_p_skew = skew(cog_to_target_frame_p);
  Eigen::Vector3d exerted_force_target_frame = exerted_wrench_target_frame.head(3) * robot_model_->getMass();
  Eigen::Vector3d compensate_torque = cog_to_target_frame_p_skew * exerted_force_target_frame;
  Eigen::Matrix3d cog_inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Matrix3d target_frame_inertia = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>();
  Eigen::Vector3d exerted_ang_acc_in_cog = cog_inertia.inverse() * (target_frame_inertia * exerted_wrench_target_frame.tail(3) + compensate_torque);

  // std::cout << "exerted ang acc in cog = [";
  // for(int i = 0; i < exerted_ang_acc_in_cog.size(); i++)
  //   {
  //     std::cout << exerted_ang_acc_in_cog(i) << " ";
  //   }
  // std::cout << std::endl;
  // std::cout << std::endl;
  // std::cout << std::endl;
  // - full_lambda_rot;

}

// void RollingController::calcAccFromTargetFrame()
// {
//   control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

//   tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
//   tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
//                            pid_controllers_.at(Y).result(),
//                            pid_controllers_.at(Z).result());
//   tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

//   Eigen::Vector3d target_force_cog = robot_model_->getMass() * Eigen::Vector3d(target_acc_cog.x(),
//                                                                                target_acc_cog.y(),
//                                                                                target_acc_cog.z());

//   KDL::Frame target_to_cog_frame = rolling_robot_model_->getTargetToCogFrame<KDL::Frame>();
//   std::cout << "target to cog frame p = [" << target_to_cog_frame.p.x() << " " << target_to_cog_frame.p.y() << " " << target_to_cog_frame.p.z() << "]" << std::endl;

//   Eigen::Vector3d target_to_cog_frame_p = Eigen::Vector3d(target_to_cog_frame.p.x(),
//                                                           target_to_cog_frame.p.y(),
//                                                           target_to_cog_frame.p.z());
//   Eigen::Matrix3d target_to_cog_frame_p_skew = skew(target_to_cog_frame_p);
//   Eigen::Vector3d force_compensate_torque = target_to_cog_frame_p_skew * target_force_cog;
//   // std::cout << "force compensate torque = [" << force_compensate_torque(0) << " " << force_compensate_torque(1) << " " << force_compensate_torque(2) << "] " << std::endl;

//   Eigen::Matrix3d cog_inertia = robot_model_->getInertia<Eigen::Matrix3d>();
//   Eigen::Matrix3d target_frame_inertia = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>();
//   Eigen::Vector3d target_torque_cog = cog_inertia * Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
//                                                                     pid_controllers_.at(PITCH).result(),
//                                                                     pid_controllers_.at(YAW).result());

//   Eigen::Vector3d target_torque_target_frame = target_torque_cog + force_compensate_torque;
//   // std::cout << "target torque in target frame: =[" << target_torque_target_frame(0) << " " << target_torque_target_frame(1) << " " << target_torque_target_frame(2) << "] " << std::endl;

//   Eigen::VectorXd target_wrench_acc_target_frame = Eigen::VectorXd::Zero(6);
//   target_wrench_acc_target_frame.head(3) = Eigen::Vector3d(target_acc_cog.x(),
//                                                            target_acc_cog.y(),
//                                                            target_acc_cog.z());
//   target_wrench_acc_target_frame.tail(3) = target_frame_inertia.inverse() * target_torque_target_frame;

//   target_wrench_acc_target_frame_ = target_wrench_acc_target_frame;

//   // Eigen::Vector3d gravity_torque_from_target_frame = rolling_robot_model_->getGravityTorqueFromTargetFrame();

// }

void RollingController::calcWrenchAllocationMatrixFromTargetFrame()
{
  /* calculate normal allocation */
  Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);

  int last_col = 0;
  std::vector<Eigen::Vector3d> rotors_origin_from_target_frame = rolling_robot_model_->getRotorsOriginFromTargetFrame<Eigen::Vector3d>();
  for(int i = 0; i < motor_num_; i++)
    {
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_target_frame.at(i));
      wrench_matrix.middleCols(last_col, 3) = wrench_map;
      last_col += 3;
    }

  Eigen::Matrix3d inertia_from_target_frame_inv = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse();
  double mass_inv = 1 / robot_model_->getMass();
  wrench_matrix.topRows(3) = mass_inv * wrench_matrix.topRows(3);
  wrench_matrix.bottomRows(3) = inertia_from_target_frame_inv * wrench_matrix.bottomRows(3);

  /* calculate masked and integrated rotaion matrix */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, 2 * motor_num_);
  const auto links_rotation_from_target_frame = rolling_robot_model_->getLinksRotationFromTargetFrame<Eigen::Matrix3d>();
  Eigen::MatrixXd mask(3, 2);
  mask << 0, 0, 1, 0, 0, 1;
  for(int i = 0; i < motor_num_; i++)
    {
      integrated_rot.block(3 * i, 2 * i, 3, 2) = links_rotation_from_target_frame.at(i) * mask;
    }

  /* calculate integarated allocation */
  full_q_mat_target_frame_ = wrench_matrix * integrated_rot;
  full_q_trans_target_frame_ = full_q_mat_target_frame_.topRows(3);
  full_q_rot_target_frame_ = full_q_mat_target_frame_.bottomRows(3);

  /* extract controlled axis */
  Eigen::MatrixXd controlled_axis_mask = Eigen::MatrixXd::Zero(control_dof_, 6);
  int last_row = 0;
  for(int i = 0; i < controlled_axis_.size(); i++)
    {
      if(controlled_axis_.at(i))
        {
          controlled_axis_mask(last_row, i) = 1;
          last_row++;
        }
    }

  // Eigen::VectorXd full_lambda = aerial_robot_model::pseudoinverse(full_q_mat) * target_wrench_acc_target_frame_get_frame_;

  // std::cout << "full_lambda = [";
  // for(int i = 0; i < full_lambda.size(); i++)
  //   {
  //     std::cout << full_lambda(i) << " ";
  //   }
  // std::cout << " ]"  << std::endl;
  // std::cout << std::endl;

}
