#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::standingPlanning()
{
  if(!start_rp_integration_)
    {
      start_rp_integration_ = true;
      spinal::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
      navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
      ROS_WARN_ONCE("start roll/pitch I control");
    }
}

void RollingController::calcStandingFullLambda()
{
  int n_variables = 2 * motor_num_;
  int n_constraints = 3 + 1 + 2 + 2 + 6;

  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n_variables, n_variables);
  Eigen::SparseMatrix<double> H_s;
  H_s = H.sparseView();

  /* normal pid result */
  Eigen::VectorXd target_wrench_target_frame;
  target_wrench_target_frame.resize(6);
  target_wrench_target_frame.tail(3) = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>() * Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                                                                                            pid_controllers_.at(PITCH).result(),
                                                                                                                            pid_controllers_.at(YAW).result());

  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_moment_from_contact_point_alined = contact_point_alined_to_cog_p_skew * robot_model_->getMass() * gravity;

  /* use sum of pid result and gravity compensation torque for attitude control */
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);

  target_wrench_target_frame.tail(3)
    = target_wrench_target_frame.tail(3)
    + gravity_compensate_ratio_ * gravity_moment_from_contact_point_alined
    + omega.cross(rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>() * omega);

  gravity_compensate_term_ = gravity_compensate_ratio_ * gravity_moment_from_contact_point_alined;

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
  A.block(8, 0, n_variables, n_variables) = Eigen::MatrixXd::Identity(n_variables, n_variables);     // in eq constraints about thrust limit

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  lower_bound.head(n_constraints - n_variables)
    <<
    target_wrench_target_frame(ROLL),
    target_wrench_target_frame(PITCH),
    target_wrench_target_frame(YAW),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound.head(n_constraints - n_variables)
    <<
    target_wrench_target_frame(ROLL),
    target_wrench_target_frame(PITCH),
    target_wrench_target_frame(YAW),
    robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);

  lower_bound.tail(n_variables) = - robot_model_->getThrustUpperLimit() * Eigen::VectorXd::Ones(n_variables);
  upper_bound.tail(n_variables) =   robot_model_->getThrustUpperLimit() * Eigen::VectorXd::Ones(n_variables);

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      for(int i = 0; i < motor_num_; i++)
        {
          upper_bound(n_constraints - n_variables + 2 * i) = -fabs(rolling_minimum_lateral_force_);
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

  solver.setPrimalVariable(full_lambda_all_);

  if(!solver.solve())
    {
      ROS_WARN_STREAM("[control][OSQP] could not solve QP!");
    }
  auto solution = solver.getSolution();
  full_lambda_all_ = solution;
  full_lambda_trans_ = solution;
}
