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

void RollingController::calcFeedbackTermForGroundControl()
{
  /* normal pid result */
  Eigen::Vector3d target_wrench_cp;
  target_wrench_cp = rolling_robot_model_->getInertiaFromControlFrame<Eigen::Matrix3d>() * Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                                                                           pid_controllers_.at(PITCH).result(),
                                                                                                           pid_controllers_.at(YAW).result());

  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);

  target_wrench_cp
    = target_wrench_cp
    + omega.cross(rolling_robot_model_->getInertiaFromControlFrame<Eigen::Matrix3d>() * omega);

  setTargetWrenchCpFbTerm(target_wrench_cp);
}

void RollingController::calcFeedforwardTermForGroundControl()
{
  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_moment_from_contact_point_alined = contact_point_alined_to_cog_p_skew * robot_model_->getMass() * gravity;

  gravity_compensate_term_ = gravity_compensate_weights_ * gravity_moment_from_contact_point_alined;
}

void RollingController::calcGroundFullLambda()
{
  auto gimbal_planning_flag = rolling_robot_model_->getGimbalPlanningFlag();
  int num_of_planned_gimbals = std::accumulate(gimbal_planning_flag.begin(), gimbal_planning_flag.end(), 0);

  int n_variables = 2 * (motor_num_ - num_of_planned_gimbals) + 1 * num_of_planned_gimbals;
  int n_constraints = 3 + 1 + 2 + 2 + n_variables; // moment(3) + force_z(1) + force_x(2) + force_y(2) + thrust limit(n_variables)

  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n_variables, n_variables);
  Eigen::SparseMatrix<double> H_s;
  H_s = H.sparseView();

  Eigen::MatrixXd planned_q_mat = rolling_robot_model_->getPlannedWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd full_q_mat = planned_q_mat;
  Eigen::MatrixXd full_q_mat_trans = full_q_mat.topRows(3);
  Eigen::MatrixXd full_q_mat_rot = full_q_mat.bottomRows(3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  int last_row = 0;
  A.block(last_row, 0, 3, n_variables) = full_q_mat_rot; last_row += 3;                              //    eq constraint about rpy torque
  A.block(last_row, 0, 1, n_variables) = full_q_mat_trans.row(Z); last_row += 1;                                       // in eq constraint about z
  A.block(last_row, 0, 1, n_variables) = full_q_mat_trans.row(X) - ground_mu_ * full_q_mat_trans.row(Z); last_row += 1;   // in eq constraint about x
  A.block(last_row, 0, 1, n_variables) = full_q_mat_trans.row(X) + ground_mu_ * full_q_mat_trans.row(Z); last_row += 1;   // in eq constraint about x
  A.block(last_row, 0, 1, n_variables) = full_q_mat_trans.row(Y) - ground_mu_ * full_q_mat_trans.row(Z); last_row += 1;  // in eq constraint about y
  A.block(last_row, 0, 1, n_variables) = full_q_mat_trans.row(Y) + ground_mu_ * full_q_mat_trans.row(Z); last_row += 1;  // in eq constraint about y
  A.block(last_row, 0, n_variables, n_variables) = Eigen::MatrixXd::Identity(n_variables, n_variables); last_row += n_variables;    // in eq constraints about thrust limit

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  Eigen::Vector3d target_wrench_cp = getTargetWrenchCpFbTerm() + getGravityCompensateTerm();

  lower_bound.head(n_constraints - n_variables)
    <<
    target_wrench_cp(0),
    target_wrench_cp(1),
    target_wrench_cp(2),
    -INFINITY,
    -ground_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY,
    -ground_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound.head(n_constraints - n_variables)
    <<
    target_wrench_cp(0),
    target_wrench_cp(1),
    target_wrench_cp(2),
    robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    ground_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    ground_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);

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

double nonlinearGroundWrenchAllocationMinObjective(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  // 0 ~ motor_num : lambda
  // motor_num ~ 2 * motor_num : gimbal roll (phi)
  // 2 * motor_num ~ 2 * motor_num + joint_num - motor_num : joint torque (if necessary)

  RollingController *controller = reinterpret_cast<RollingController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  double ret = 0;
  int motor_num = robot_model_for_control->getRotorNum();

  const std::vector<double>& opt_initial_x = controller->getOptInitialX();
  const std::vector<double>& opt_cost_weights = controller->getOptCostWeights();
  const std::vector<double>& current_gimbal_angles = robot_model_for_control->getCurrentGimbalAngles();
  int ground_navigation_mode = controller->getGroundNavigationMode();

  // objective
  // thrust and gimbal part
  for(int i = 0; i < motor_num; i++)
    {
      ret += opt_cost_weights.at(0) * x.at(i) * x.at(i);
      ret += opt_cost_weights.at(1) * (x.at(i + motor_num) - opt_initial_x.at(i + motor_num)) * (x.at(i + motor_num) - opt_initial_x.at(i + motor_num));
      ret += opt_cost_weights.at(2) * (x.at(i + motor_num) - angles::normalize_angle(current_gimbal_angles.at(i))) * (x.at(i + motor_num) - angles::normalize_angle(current_gimbal_angles.at(i)));
      if(ground_navigation_mode == aerial_robot_navigation::ROLLING_STATE)
        {
          ret += opt_cost_weights.at(3) * (x.at(i + motor_num) - M_PI / 2.0) * (x.at(i + motor_num) - M_PI / 2.0);
        }
    }

  // joint torque part
  const double torque_weight = controller->getOptJointTorqueWeight();
  if(x.size() > 2 * motor_num)
    {
      for(int i = 0; i < x.size() - 2 * motor_num; i++)
        {
          ret += torque_weight * x.at(i + 2 * motor_num) * x.at(i + 2 * motor_num);
        }
    }

  if(grad.empty()) return ret;

  // set gradient
  // thrust and gimbal part
  for(int i = 0; i < motor_num; i++)
    {
      grad.at(i)             =  opt_cost_weights.at(0) * 2 * x.at(i);
      grad.at(i + motor_num) =  opt_cost_weights.at(1) * 2 * (x.at(i + motor_num) - opt_initial_x.at(i + motor_num));
      grad.at(i + motor_num) += opt_cost_weights.at(2) * 2 * (x.at(i + motor_num) - angles::normalize_angle(current_gimbal_angles.at(i)));
      if(ground_navigation_mode == aerial_robot_navigation::ROLLING_STATE)
        {
          grad.at(i + motor_num) += opt_cost_weights.at(3) * 2 * (x.at(i + motor_num) - M_PI / 2.0);
        }
    }

  // joint torque part
  for(int i = 0; i < x.size() - 2 * motor_num; i++)
    {
      grad.at(i + 2 * motor_num) = torque_weight * 2 * x.at(i + 2 * motor_num);
    }

  return ret;
}

void nonlinearGroundWrenchAllocationEqConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* ptr)
{
  // 0 ~ 3 : target_wrench_control_frame - Q * lambda = 0

  RollingController *controller = reinterpret_cast<RollingController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  std::vector<Eigen::Matrix3d> links_rotation_from_control_frame = robot_model_for_control->getLinksRotationFromControlFrame<Eigen::Matrix3d>();
  std::vector<Eigen::Vector3d> rotors_origin_from_control_frame = robot_model_for_control->getRotorsOriginFromControlFrame<Eigen::Vector3d>();
  std::vector<double> rotor_tilt = controller->getRotorTilt();

  double m_f_rate = robot_model_for_control->getMFRate();
  std::map<int, int> rotor_direction = robot_model_for_control->getRotorDirection();

  Eigen::Vector3d target_wrench_cp = controller->getTargetWrenchCpFbTerm() + controller->getGravityCompensateTerm();

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num);
  for(int i = 0; i < motor_num; i++) lambda(i) = x[i];

  Eigen::MatrixXd Q_rot = Eigen::MatrixXd::Zero(3, motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d u_Li;
      u_Li <<
         sin(rotor_tilt.at(i)),
        -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
         cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

      Q_rot.block(0, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_control_frame.at(i)) + m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) * links_rotation_from_control_frame.at(i) * u_Li;
    }

  Eigen::VectorXd torque_diff = Q_rot * lambda - target_wrench_cp;
  for(int i = 0; i < m; i++) result[i] = torque_diff(i);

  if(grad == NULL) return;

  // thrust part
  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < motor_num; j++)
        {
          grad[i * n + j] = Q_rot(i, j);
        }
    }

  // gimbal part
  Eigen::MatrixXd dtau_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d du_Li_dphi;
      du_Li_dphi <<
        0,
       -cos(x[i + motor_num]) * cos(rotor_tilt.at(i)),
       -sin(x[i + motor_num]) * cos(rotor_tilt.at(i));

      dtau_dphi.block(0, i, 3, 1) =  (aerial_robot_model::skew(rotors_origin_from_control_frame.at(i)) + m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) * links_rotation_from_control_frame.at(i) * du_Li_dphi * x[i];
    }

  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < motor_num; j++)
        {
          grad[i * n + (j + motor_num)] = dtau_dphi(i, j);
        }
    }

  // joint torque part
  for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < n - 2 * motor_num; j++)
        {
          grad[i * n + (j + 2 * motor_num)] = 0;
        }
    }
}

void nonlinearGroundWrenchAllocationInEqConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* ptr)
{
  // 0:    f_z < mg -> f_z - mg < 0
  // 1, 2: -mu * (mg - f_z) < f_x < mu * (mg - f_z) -> f_x + mu * f_z - mu * mg < 0, -f_x + mu * f_z - mu * mg < 0
  // 3, 4: -mu * (mg - f_z) < f_y < mu * (mg - f_z) -> f_y + mu * f_z - mu * mg < 0, -f_y + mu * f_z - mu * mg < 0

  RollingController *controller = reinterpret_cast<RollingController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  std::vector<Eigen::Matrix3d> links_rotation_from_control_frame = robot_model_for_control->getLinksRotationFromControlFrame<Eigen::Matrix3d>();
  std::vector<double> rotor_tilt = controller->getRotorTilt();

  double m_f_rate = robot_model_for_control->getMFRate();
  std::map<int, int> rotor_direction = robot_model_for_control->getRotorDirection();

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num);
  for(int i = 0; i < motor_num; i++) lambda(i) = x[i];

  Eigen::MatrixXd Q_trans = Eigen::MatrixXd::Zero(3, motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d u_Li;
      u_Li <<
         sin(rotor_tilt.at(i)),
        -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
         cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

      Q_trans.block(0, i, 3, 1) = links_rotation_from_control_frame.at(i) * u_Li;
    }

  double ground_mu = controller->getGroundMu();
  Eigen::VectorXd exerted_force = Q_trans * lambda;
  result[0] =  exerted_force(2) - robot_model_for_control->getMass() * robot_model_for_control->getGravity()(2);
  result[1] =  exerted_force(0) + ground_mu * exerted_force(2) - ground_mu * robot_model_for_control->getMass() * robot_model_for_control->getGravity()(2);
  result[2] = -exerted_force(0) + ground_mu * exerted_force(2) - ground_mu * robot_model_for_control->getMass() * robot_model_for_control->getGravity()(2);
  result[3] =  exerted_force(1) + ground_mu * exerted_force(2) - ground_mu * robot_model_for_control->getMass() * robot_model_for_control->getGravity()(2);
  result[4] = -exerted_force(1) + ground_mu * exerted_force(2) - ground_mu * robot_model_for_control->getMass() * robot_model_for_control->getGravity()(2);

  if(grad == NULL) return;

  // thrust part
  for(int i = 0; i < motor_num; i++)
    {
      grad[0 * n + i] =  Q_trans(2, i);
      grad[1 * n + i] =  Q_trans(0, i) + ground_mu * Q_trans(2, i);
      grad[2 * n + i] = -Q_trans(0, i) + ground_mu * Q_trans(2, i);
      grad[3 * n + i] =  Q_trans(1, i) + ground_mu * Q_trans(2, i);
      grad[4 * n + i] = -Q_trans(1, i) + ground_mu * Q_trans(2, i);
    }

  // gimbal part
  Eigen::MatrixXd df_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d du_Li_dphi;
      du_Li_dphi <<
        0,
       -cos(x[i + motor_num]) * cos(rotor_tilt.at(i)),
       -sin(x[i + motor_num]) * cos(rotor_tilt.at(i));

      df_dphi.block(0, i, 3, 1) = links_rotation_from_control_frame.at(i) * du_Li_dphi * x[i];
    }

  for(int i = 0; i < motor_num; i++)
    {
      grad[0 * n + (i + motor_num)] =  df_dphi(2, i);
      grad[1 * n + (i + motor_num)] =  df_dphi(0, i) + ground_mu * df_dphi(2, i);
      grad[2 * n + (i + motor_num)] = -df_dphi(0, i) + ground_mu * df_dphi(2, i);
      grad[3 * n + (i + motor_num)] =  df_dphi(1, i) + ground_mu * df_dphi(2, i);
      grad[4 * n + (i + motor_num)] = -df_dphi(1, i) + ground_mu * df_dphi(2, i);
    }

  // joint torque part
  for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < n - 2 * motor_num; j++)
        {
          grad[i * n + (j + 2 * motor_num)] = 0;
        }
    }
}

void RollingController::nonlinearGroundWrenchAllocation()
{
  int n_variables;
  if(opt_add_joint_torque_constraints_)
    n_variables = 2 * motor_num_ + robot_model_->getJointNum() - motor_num_;
  else
    n_variables = 2 * motor_num_;

  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray joint_positions = robot_model_->getJointPositions();

  if(first_run_)
    {
      opt_x_prev_.resize(n_variables, 0.0);
      opt_initial_x_.resize(n_variables, 0.0);
      nlopt_log_.resize(n_variables, 0.0);
      robot_model_for_control_->updateRobotModel(joint_positions);
      robot_model_for_control_->calcContactPoint();
      ROS_INFO_STREAM("calc contact point once");
    }

  robot_model_for_control_->updateRobotModel(joint_positions);

  if(n_variables > 2 * motor_num_) jointTorquePreComputation();

  nlopt::opt slsqp_solver(nlopt::LD_SLSQP, n_variables);
  slsqp_solver.set_min_objective(nonlinearGroundWrenchAllocationMinObjective, this);
  slsqp_solver.add_equality_mconstraint(nonlinearGroundWrenchAllocationEqConstraints, this, {1e-6, 1e-6, 1e-6});
  slsqp_solver.add_inequality_mconstraint(nonlinearGroundWrenchAllocationInEqConstraints, this, {1e-6, 1e-6, 1e-6, 1e-6, 1e-6});
  if(n_variables > 2 * motor_num_)
    {
      if(first_run_) ROS_INFO_STREAM("[control] add constraint about joint torque");
      slsqp_solver.add_equality_mconstraint(nonlinearWrenchAllocationTorqueConstraints, this, std::vector<double>(n_variables - 2 * motor_num_, 1e-4));
    }

  /* set bounds */
  std::vector<double> lb(n_variables, -INFINITY);
  std::vector<double> ub(n_variables, INFINITY);
  for(int i = 0; i < motor_num_; i++)
    {
      lb.at(i) = 0;
      ub.at(i) = robot_model_->getThrustUpperLimit();
      lb.at(i + motor_num_) = -M_PI;
      ub.at(i + motor_num_) =  M_PI;
      if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
        {
          lb.at(i + motor_num_) = 0;
          ub.at(i + motor_num_) = M_PI;
        }
    }

  slsqp_solver.set_lower_bounds(lb);
  slsqp_solver.set_upper_bounds(ub);
  slsqp_solver.set_xtol_rel(1e-6);
  slsqp_solver.set_ftol_rel(1e-8);
  slsqp_solver.set_maxeval(1000);

  /* set initial variable */
  std::vector<double> opt_x(n_variables);
  // thrust and gimbal part (use osqp solution)
  for(int i = 0; i < motor_num_; i++)
    {
      opt_x.at(i) = std::clamp((double)full_lambda_all_.segment(2 * i, 2).norm() / fabs(cos(rotor_tilt_.at(i))),
                               lb.at(i),
                               ub.at(i));
      opt_x.at(i + motor_num_) = std::clamp((double)angles::normalize_angle(atan2(-full_lambda_all_(2 * i + 0), full_lambda_all_(2 * i + 1))),
                                            lb.at(i + motor_num_),
                                            ub.at(i + motor_num_));
    }

  // joint torque part (use previous solution)
  for(int i = 0; i < n_variables - 2 * motor_num_; i++)
    {
      opt_x.at(2 * motor_num_ + i) = opt_x_prev_.at(2 * motor_num_ + i);
    }

  for(int i = 0; i < n_variables; i++)
    opt_initial_x_.at(i) = opt_x.at(i);

  /* solve optimization problem */
  double max_val;
  nlopt::result result;
  try
    {
      result = slsqp_solver.optimize(opt_x, max_val);
    }
  catch(std::runtime_error error)
    {}

  if(result < 0) ROS_ERROR_STREAM_THROTTLE(1.0, "[nlopt] failed to solve. result is " << result);

  for(int i = 0; i < opt_x.size(); i++) nlopt_log_.at(i) = opt_x.at(i);

  /* set optimal variables to actuator input */
  for(int i = 0; i < motor_num_; i++)
    {
      lambda_all_.at(i) = opt_x.at(i);
      lambda_trans_.at(i) = opt_x.at(i);
      target_gimbal_angles_.at(i) = opt_x.at(i + motor_num_);
    }

  for(int i = 0; i < n_variables; i++)
    opt_x_prev_.at(i) = opt_x.at(i);
}

void RollingController::cfgNloptCallback(delta::nloptConfig &config, uint32_t level)
{
  using Levels = delta::DynamicReconfigureLevels;
  switch(level)
    {
    case Levels::RECONFIGURE_THRUST_WEIGHT:
      opt_cost_weights_.at(level) = config.thrust_weight;
      ROS_INFO_STREAM("change weight at " << level << " to " << opt_cost_weights_.at(level));
      break;

    case Levels::RECONFIGURE_GIMBAL_LINEAR_SOLUTION_DIST_WEIGHT:
      opt_cost_weights_.at(level) = config.gimbal_linear_solution_dist_weight;
      ROS_INFO_STREAM("change weight at " << level << " to " << opt_cost_weights_.at(level));
      break;

    case Levels::RECONFIGURE_GIMBAL_CURRENT_ANGLE_DIST_WEIGHT:
      opt_cost_weights_.at(level) = config.gimbal_current_angle_dist_weight;
      ROS_INFO_STREAM("change weight at " << level << " to " << opt_cost_weights_.at(level));
      break;

    case Levels::RECONFIGURE_GIMBAL_CENTER_DIST_WEIGHT:
      opt_cost_weights_.at(level) = config.gimbal_center_dist_weight;
      ROS_INFO_STREAM("change weight at " << level << " to " << opt_cost_weights_.at(level));
      break;

    case Levels::RECONFIGURE_JOINT_TORQUE_WEIGHT:
      opt_joint_torque_weight_ = config.joint_torque_weight;
      ROS_INFO_STREAM("change weight at " << level << " to " << opt_joint_torque_weight_);
      break;

    default:
      break;
    }
}
