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
  auto gimbal_planning_flag = rolling_robot_model_->getGimbalPlanningFlag();
  int num_of_planned_gimbals = std::accumulate(gimbal_planning_flag.begin(), gimbal_planning_flag.end(), 0);

  int n_variables = 2 * (motor_num_ - num_of_planned_gimbals) + 1 * num_of_planned_gimbals;
  int n_constraints = 3 + 1 + 2 + 2 + n_variables;

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
    + gravity_compensate_weights_ * gravity_moment_from_contact_point_alined
    + omega.cross(rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>() * omega);

  gravity_compensate_term_ = gravity_compensate_weights_ * gravity_moment_from_contact_point_alined;

  Eigen::MatrixXd planned_q_mat = rolling_robot_model_->getPlannedWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd full_q_mat = planned_q_mat;
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

  int last_col = n_constraints - n_variables;
  for(int i = 0; i < motor_num_; i++)
    {
      if(gimbal_planning_flag.at(i))
        {
          lower_bound(last_col) = 0.0;
          upper_bound(last_col) = robot_model_->getThrustUpperLimit();

          last_col += 1;
        }
      else
        {
          lower_bound.segment(last_col, 2) = - robot_model_->getThrustUpperLimit() * Eigen::VectorXd::Ones(2);
          upper_bound.segment(last_col, 2) =   robot_model_->getThrustUpperLimit() * Eigen::VectorXd::Ones(2);

          if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
            {
              upper_bound(last_col) = -fabs(rolling_minimum_lateral_force_);
            }

          last_col += 2;
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

  /* set primal variable from previous solution x*/
  last_col = 0;
  Eigen::VectorXd initial_variable = Eigen::VectorXd::Zero(n_variables);
  for(int i = 0; i < motor_num_; i++)
    {
      if(gimbal_planning_flag.at(i))
        {
          initial_variable(last_col) = lambda_all_.at(i);
          last_col += 1;
        }
      else
        {
          initial_variable.segment(last_col, 2) = full_lambda_all_.segment(2 * i, 2);
          last_col += 2;
        }
    }
  solver.setPrimalVariable(initial_variable);

  if(!solver.solve())
    {
      ROS_WARN_STREAM("[control][OSQP] could not solve QP!");
    }
  auto solution = solver.getSolution();

  /* reconstruct full lambda */
  Eigen::VectorXd full_lambda = Eigen::VectorXd::Zero(2 * motor_num_);
  last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      if(gimbal_planning_flag.at(i))
        {
          full_lambda.segment(2 * i, 2) = solution(last_col) * Eigen::VectorXd::Ones(2);
          last_col += 1;
        }
      else
        {
          full_lambda.segment(2 * i, 2) = solution.segment(last_col, 2);
          last_col += 2;
        }
    }

  full_lambda_all_ = full_lambda;
  full_lambda_trans_ = full_lambda;
}
