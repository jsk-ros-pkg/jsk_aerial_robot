#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::standingPlanning()
{
  /* set target roll of baselink */
  tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);

  double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
  double baselink_pitch = estimator_->getEuler(Frame::BASELINK, estimate_mode_).y();
  double target_baselink_roll = rolling_navigator_->getCurrTargetBaselinkRotRoll();
  double target_baselink_pitch = rolling_navigator_->getCurrTargetBaselinkRotPitch();

  if(!start_rp_integration_)
    {
      start_rp_integration_ = true;
      spinal::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
      navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
      ROS_WARN_ONCE("start roll/pitch I control");
    }

  /* get target ang vel and desired baselink pitch from rolling navigator */


  /* set target ang vel and desired baselink pitch to navigator and spinal */


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

  if(standing_baselink_pitch_update_)
    {
      if(ros::Time::now().toSec() - standing_baselink_ref_pitch_last_update_time_ > standing_baselink_ref_pitch_update_thresh_)
        {
          standing_baselink_ref_pitch_last_update_time_ = ros::Time::now().toSec();
          rolling_navigator_->setFinalTargetBaselinkRotPitch(baselink_pitch);
        }
    }

  double du = ros::Time::now().toSec() - rolling_control_timestamp_;
  if(!rolling_navigator_->getPitchAngVelUpdating())
    {
      rolling_navigator_->setBaselinkRotForceUpdateMode(false);
      rolling_navigator_->setCurrentTargetBaselinkRotPitch(target_baselink_pitch);
      navigator_->setTargetOmegaY(0);
    }
  else
    {
      rolling_navigator_->setBaselinkRotForceUpdateMode(true);
      double target_pitch_ang_vel = rolling_navigator_->getTargetPitchAngVel();
      rolling_navigator_->setCurrentTargetBaselinkRotPitch(target_baselink_pitch + du * target_pitch_ang_vel);

      navigator_->setTargetOmegaY(target_pitch_ang_vel);

      rpy_ = estimator_->getEuler(Frame::COG, estimate_mode_);
      omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);
      target_rpy_ = navigator_->getTargetRPY();
      target_omega_ = navigator_->getTargetOmega();

      pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du, target_omega_.y() - omega_.y());

    }
  rolling_control_timestamp_ = ros::Time::now().toSec();

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

  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = robot_model_->getMass() * contact_point_alined_to_cog_p_skew * gravity;

  /* use sum of pid result and gravity compensation torque for attitude control */
  target_wrench_acc_target_frame.tail(3) = target_wrench_acc_target_frame.tail(3) + gravity_compensate_ratio_ * gravity_ang_acc_from_contact_point_alined;
  target_wrench_acc_target_frame_ = target_wrench_acc_target_frame;

  Eigen::MatrixXd full_q_mat_trans = robot_model_->getMass() * full_q_trans_target_frame_;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.topRows(3) = full_q_rot_target_frame_;                                                           //    eq constraint about rpy torque
  A.block(3, 0, 1, n_variables) = full_q_mat_trans.row(Z);                                           // in eq constraint about z
  A.block(4, 0, 1, n_variables) = full_q_mat_trans.row(X) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(5, 0, 1, n_variables) = full_q_mat_trans.row(X) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(6, 0, 1, n_variables) = full_q_mat_trans.row(Y) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y
  A.block(7, 0, 1, n_variables) = full_q_mat_trans.row(Y) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  lower_bound
    <<
    target_wrench_acc_target_frame(ROLL) - epsilon,
    target_wrench_acc_target_frame(PITCH) - epsilon,
    target_wrench_acc_target_frame(YAW) - epsilon,
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound <<
    target_wrench_acc_target_frame(ROLL) + epsilon,
    target_wrench_acc_target_frame(PITCH) + epsilon,
    target_wrench_acc_target_frame(YAW) + epsilon,
    robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);


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

  slsqpSolver slsqp_solver = slsqpSolver(n_variables, n_constraints,
                                         target_wrench_acc_target_frame, full_q_mat_target_frame_,
                                         Eigen::VectorXd::Ones(n_variables) * 4.0,
                                         robot_model_->getMass(), robot_model_->getGravity()(Z), steering_mu_);
  slsqp_solver.solve();
}

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
}
