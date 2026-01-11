#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void DeltaController::calcAccFromCog()
{
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) *
                          tf::Matrix3x3(cog2baselink_rot).inverse();  // w_R_b * cog_R_b.inverse() = w_R_cog
                        
  double x_term = pid_controllers_.at(X).result();
  double y_term = pid_controllers_.at(Y).result();
  double xy_term = std::sqrt((x_term * x_term + y_term * y_term));
  if (xy_term > 4.0){
    x_term = x_term * 4.0 / xy_term;
    y_term = y_term * 4.0 / xy_term;
    std::cout << x_term << ", " << y_term << std::endl;
  }

  tf::Vector3 target_linear_acc_w(x_term, y_term,
                                  pid_controllers_.at(Z).result());
  tf::Vector3 target_linear_acc_cog = cog_rot.inverse() * target_linear_acc_w;

  Eigen::VectorXd target_acc_cog = Eigen::VectorXd::Zero(6);

  target_acc_cog.head(3) =
      Eigen::Vector3d(target_linear_acc_cog.x(), target_linear_acc_cog.y(), target_linear_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  target_acc_cog.tail(3) =
      Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + inertia.inverse() * gyro;

  target_acc_cog_ = target_acc_cog;
}

void DeltaController::forceLandingProcess()
{
  if (navigator_->getForceLandingFlag() &&
      pid_controllers_.at(Z).result() < 5.0)  // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;
}

void DeltaController::wrenchAllocation()
{
  if (ros::Time::now().toSec() - full_q_mat_update_stamp_ > torque_allocation_matrix_inv_pub_interval_)
  {
    full_q_mat_update_stamp_ = ros::Time::now().toSec();
    full_q_mat_ = robot_model_for_control_->getFullWrenchAllocationMatrixFromCog();
    full_q_mat_inv_ = aerial_robot_model::pseudoinverse(full_q_mat_);
  }

  if (linear_mode_ || first_run_)
  {
    linearWrenchAllocation();
  }
  else
  {
    nonlinearWrenchAllocation();

    if (nlopt_result_ < nlopt::SUCCESS || nlopt::MAXEVAL_REACHED <= nlopt_result_)
    {
      ROS_WARN_THROTTLE(1.0, "[DeltaController] nlopt failed to solve, use linear allocation result");
      linearWrenchAllocation();
    }
  }
}

void DeltaController::linearWrenchAllocation()
{
  Eigen::VectorXd target_wrench_cog = target_acc_cog_;
  target_wrench_cog.head(3) = robot_model_->getMass() * target_wrench_cog.head(3);
  target_wrench_cog.tail(3) = robot_model_->getInertia<Eigen::Matrix3d>() * target_wrench_cog.tail(3);

  int dof = motor_on_rigid_frame_num_ * 2 + motor_on_soft_frame_num_;
  Eigen::MatrixXd weight = Eigen::MatrixXd::Identity(dof, dof);
  for (int i = 0; i < motor_on_rigid_frame_num_; i++)
  {
    weight(2 * i + 0, 2 * i + 0) = 1.0;
    weight(2 * i + 1, 2 * i + 1) = 0.1;
  }
  for (int i = 0; i < motor_on_soft_frame_num_; i++)
  {
    weight(2 * motor_on_rigid_frame_num_ + i, 2 * motor_on_rigid_frame_num_ + i) = 0.1;
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dof, dof);
  H.diagonal() = 2.0 * weight.diagonal();
  Eigen::VectorXd g = -2.0 * weight * prev_target_vectoring_f_;

  // H.diagonal().setConstant(2.0);
  // Eigen::VectorXd g = prev_target_vectoring_f_ * -2.0;

  int n_constraints = 6 + motor_on_rigid_frame_num_ * 2 + motor_on_soft_frame_num_;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, dof);
  A.block(0, 0, 6, dof) = full_q_mat_;
  for (int i = 0; i < motor_on_rigid_frame_num_; i++)
  {
    A(6 + 2 * i + 0, 2 * i + 0) = 1.0;
    A(6 + 2 * i + 1, 2 * i + 1) = 1.0;
  } 

  for (int i = 0; i < motor_on_soft_frame_num_; i++)
  {
    A(6 + 2 * motor_on_rigid_frame_num_ + i, 2 * motor_on_rigid_frame_num_ + i) = 1.0;
  }

  Eigen::VectorXd lb(n_constraints);
  Eigen::VectorXd ub(n_constraints);

  lb.head(6) = target_wrench_cog;
  ub.head(6) = target_wrench_cog;

  // double yaw_margin = 0.1;
  // lb(5) = target_wrench_cog(5) - yaw_margin;
  // ub(5) = target_wrench_cog(5) + yaw_margin;

  // double x_margin = 0.1;
  // lb(0) = target_wrench_cog(0) - x_margin;
  // ub(0) = target_wrench_cog(0) + x_margin;

  // double y_margin = 0.1;
  // lb(1) = target_wrench_cog(1) - y_margin;
  // ub(1) = target_wrench_cog(1) + y_margin;

  for (int i = 0; i < motor_on_rigid_frame_num_; i++)
  {
    lb(6 + 2 * i + 0) = -robot_model_->getThrustUpperLimit(i);
    ub(6 + 2 * i + 0) = robot_model_->getThrustUpperLimit(i);
    lb(6 + 2 * i + 1) = robot_model_->getThrustLowerLimit(i);
    ub(6 + 2 * i + 1) = robot_model_->getThrustUpperLimit(i);
  }

  for (int i = 0; i < motor_on_soft_frame_num_; i++)
  {
    lb(6 + 2 * motor_on_rigid_frame_num_ + i) = robot_model_->getThrustLowerLimit(motor_on_rigid_frame_num_ + i);
    ub(6 + 2 * motor_on_rigid_frame_num_ + i) = robot_model_->getThrustUpperLimit(motor_on_rigid_frame_num_ + i);
  }

  Eigen::SparseMatrix<double> H_s = H.sparseView();
  Eigen::SparseMatrix<double> A_s = A.sparseView();
  if(!target_vectoring_qp_solver_.isInitialized())
  {
      target_vectoring_qp_solver_.settings()->setVerbosity(false);
      target_vectoring_qp_solver_.settings()->setWarmStart(true);
      target_vectoring_qp_solver_.settings()->setPolish(false);
      target_vectoring_qp_solver_.settings()->setMaxIteraction(1000);
      target_vectoring_qp_solver_.settings()->setAbsoluteTolerance(1e-4);
      target_vectoring_qp_solver_.settings()->setRelativeTolerance(1e-4);

      target_vectoring_qp_solver_.data()->setNumberOfVariables(dof);
      target_vectoring_qp_solver_.data()->setNumberOfConstraints(n_constraints);
      target_vectoring_qp_solver_.data()->setHessianMatrix(H_s);
      target_vectoring_qp_solver_.data()->setGradient(g);
      target_vectoring_qp_solver_.data()->setLinearConstraintsMatrix(A_s);
      target_vectoring_qp_solver_.data()->setLowerBound(lb);
      target_vectoring_qp_solver_.data()->setUpperBound(ub);
      target_vectoring_qp_solver_.initSolver();
  }
  else
  {
      target_vectoring_qp_solver_.updateHessianMatrix(H_s);
      target_vectoring_qp_solver_.updateGradient(g);
      target_vectoring_qp_solver_.updateLinearConstraintsMatrix(A_s);
      target_vectoring_qp_solver_.updateBounds(lb, ub);
  }

  Eigen::VectorXd full_lambda = Eigen::VectorXd::Zero(dof);
  bool solved = target_vectoring_qp_solver_.solve();
  if(solved){
    full_lambda = target_vectoring_qp_solver_.getSolution();
  } else {
    std::cout << "Warning: QP solver failed to solve!" << std::endl;
    full_lambda = full_q_mat_inv_ * target_wrench_cog;
    full_lambda.noalias() += prev_target_vectoring_f_;
    full_lambda.noalias() -= full_q_mat_inv_ * (full_q_mat_ * prev_target_vectoring_f_);
  }
  prev_target_vectoring_f_ = full_lambda;

  for (int i = 0; i < motor_on_rigid_frame_num_; i++)
  {
    lambda_all_.at(i) =
        std::clamp((float)full_lambda.segment(2 * i, 2).norm(), (float)robot_model_->getThrustLowerLimit(),
                   (float)robot_model_->getThrustUpperLimit());
    target_gimbal_angles_.at(i) = angles::normalize_angle(atan2(-full_lambda(2 * i + 0), full_lambda(2 * i + 1)));
  }

  // fail_safe
  target_gimbal_angles_.at(1) = std::clamp(target_gimbal_angles_.at(1), -1.0, 3.14);

  for (int i = 0; i < motor_on_soft_frame_num_; i++)
  {
    lambda_all_.at(motor_on_rigid_frame_num_+i) = std::clamp((float)full_lambda(2*motor_on_rigid_frame_num_+i), (float)robot_model_->getThrustLowerLimit(),
                                       (float)robot_model_->getThrustUpperLimit());
  }

  std_msgs::Float64MultiArray msg;
  msg.data.resize(full_lambda.size());
  for (unsigned int i = 0; i < full_lambda.size(); i++)
  {
    msg.data[i] = full_lambda(i);
  }
  full_lambda_pub_.publish(msg);
}

void DeltaController::nonlinearWrenchAllocation()
{
  int n_variables = 2 * motor_num_;

  nlopt_log_.resize(n_variables, 0.0);
  nlopt_iterations_ = 0;

  nlopt::opt slsqp_solver(nlopt::LD_SLSQP, n_variables);
  slsqp_solver.set_min_objective(nonlinearWrenchAllocationMinObjective, this);
  slsqp_solver.add_equality_mconstraint(nonlinearWrenchAllocationEqConstraints, this,
                                        { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 });

  std::vector<double> lb(n_variables, -INFINITY);
  std::vector<double> ub(n_variables, INFINITY);
  for (int i = 0; i < motor_num_; i++)
  {
    lb.at(i) = 0;
    ub.at(i) = robot_model_->getThrustUpperLimit();
    lb.at(i + motor_num_) = -M_PI;
    ub.at(i + motor_num_) = M_PI;
  }

  slsqp_solver.set_lower_bounds(lb);
  slsqp_solver.set_upper_bounds(ub);
  slsqp_solver.set_xtol_rel(1e-6);
  slsqp_solver.set_ftol_rel(1e-6);
  slsqp_solver.set_maxeval(1000);

  /* set initial variable */
  std::vector<double> opt_x(n_variables, 0);
  for (int i = 0; i < motor_num_; i++)
  {
    opt_x.at(i) = std::clamp((double)lambda_all_.at(i), lb.at(i), ub.at(i));
    opt_x.at(i + motor_num_) = std::clamp((double)angles::normalize_angle(target_gimbal_angles_.at(i)),
                                          lb.at(i + motor_num_), ub.at(i + motor_num_));
  }

  double max_val;
  nlopt::result result;
  try
  {
    result = slsqp_solver.optimize(opt_x, max_val);
    nlopt_result_ = result;
  }
  catch (std::runtime_error error)
  {
  }

  if (result < 0)
    ROS_ERROR_STREAM("[nlopt] failed to solve. result is " << result);

  for (int i = 0; i < opt_x.size(); i++)
    nlopt_log_.at(i) = opt_x.at(i);

  /* set optimal variables to actuator input */
  for (int i = 0; i < motor_num_; i++)
  {
    lambda_all_.at(i) = opt_x.at(i);
    target_gimbal_angles_.at(i) = opt_x.at(i + motor_num_);
  }
}
