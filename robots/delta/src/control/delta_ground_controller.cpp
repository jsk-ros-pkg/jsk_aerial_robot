#include <delta/control/delta_controller.h>

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

  if(standing_baselink_pitch_update_ && ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE)
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

  int standing_mode_n_constraints = n_constraints;
  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      n_constraints += 3;
    }

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
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse() * contact_point_alined_to_cog_p_skew * robot_model_->getMass() * gravity;

  /* use sum of pid result and gravity compensation torque for attitude control */
  target_wrench_acc_target_frame.tail(3) = target_wrench_acc_target_frame.tail(3) + gravity_compensate_ratio_ * gravity_ang_acc_from_contact_point_alined;

  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd full_q_mat_trans = full_q_mat.topRows(3);
  Eigen::MatrixXd full_q_mat_rot = full_q_mat.bottomRows(3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.topRows(3) = full_q_mat_rot;                                                           //    eq constraint about rpy torque
  A.block(3, 0, 1, n_variables) = full_q_mat_trans.row(Z);                                           // in eq constraint about z
  A.block(4, 0, 1, n_variables) = full_q_mat_trans.row(X) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(5, 0, 1, n_variables) = full_q_mat_trans.row(X) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(6, 0, 1, n_variables) = full_q_mat_trans.row(Y) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y
  A.block(7, 0, 1, n_variables) = full_q_mat_trans.row(Y) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      A.block(8, 0, 1, n_variables)  << 1, 0, 0, 0, 0, 0;
      A.block(9, 0, 1, n_variables)  << 0, 0, 1, 0, 0, 0;
      A.block(10, 0, 1, n_variables) << 0, 0, 0, 0, 1, 0;
    }

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  lower_bound.head(standing_mode_n_constraints)
    <<
    target_wrench_acc_target_frame(ROLL) - epsilon,
    target_wrench_acc_target_frame(PITCH) - epsilon,
    target_wrench_acc_target_frame(YAW) - epsilon,
    -INFINITY,
    -steering_mu_ * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound.head(standing_mode_n_constraints)
    <<
    target_wrench_acc_target_frame(ROLL) + epsilon,
    target_wrench_acc_target_frame(PITCH) + epsilon,
    target_wrench_acc_target_frame(YAW) + epsilon,
    robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getGravity()(Z);

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      for(int i = 0; i < motor_num_; i++)
        {
          lower_bound(standing_mode_n_constraints + i) = -robot_model_->getThrustUpperLimit();
          upper_bound(standing_mode_n_constraints + i) = -fabs(rolling_minimum_lateral_force_);
        }
    }

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
      ROS_ERROR("[control][OSQP] init solver error!");
    }

  if(!solver.solve())
    {
      ROS_WARN_STREAM("[control][OSQP] could not reach the solution.");
      rolling_navigator_->setGroundNavigationMode(aerial_robot_navigation::STANDING_STATE);
    }

  auto solution = solver.getSolution();

  full_lambda_all_ = solution;
  full_lambda_trans_ = solution;
}
