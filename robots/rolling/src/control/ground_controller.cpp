#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::hoge()
{
  std::cout << "hoge" << std::endl;
}

void RollingController::steeringControlWrenchAllocation()
{
  nlopt::opt nl_solver(nlopt::LD_SLSQP, 2 * motor_num_);
  nl_solver.set_min_objective(steeringControlWrenchAllocationObject, this);
  nl_solver.add_equality_constraint(steeringControlWrenchAllocationConstraint, this, 1e-2);
  nl_solver.set_xtol_rel(1e-4);
  nl_solver.set_maxeval(1000);

  std::vector<double> lb(2 * motor_num_);
  std::vector<double> ub(2 * motor_num_);
  for(int i = 0; i < 2 * motor_num_; i++)
    {
      lb.at(i) = -25.0;
      ub.at(i) = 25.0;
    }
  nl_solver.set_lower_bounds(lb);
  nl_solver.set_upper_bounds(ub);

  std::vector<double> full_lambda(2 * motor_num_, 0.0);
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat_);
  Eigen::VectorXd initial_value = full_q_mat_inv * target_wrench_acc_cog_;
  std::cout << "desired value" << std::endl;
  std::cout << initial_value << std::endl;

  for(int i = 0; i < initial_value.size(); i++)
    {
      full_lambda.at(i) = initial_value(i) - 1.0;
    }
  std::cout << "initial value" << std::endl;
  for(int i = 0; i < full_lambda.size(); i++)
    {
      std::cout << full_lambda.at(i) << " ";
    }
  std::cout << std::endl;

  double diff;
  nlopt::result result;

  try
    {
      result = nl_solver.optimize(full_lambda, diff);
    }
  catch(nlopt::roundoff_limited)
    {
      ROS_WARN_STREAM("[control][nlopt] catch nlopt::roundoff_limited");
    }

  if(result != nlopt::SUCCESS)
    {
      ROS_WARN_STREAM("the optimize solution does not succeed, result is " << result);
    }

  std::cout << "optimized value" << std::endl;
  for(int i = 0; i < full_lambda.size(); i++)
    {
      std::cout << full_lambda.at(i) << " ";
    }
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;

}

double steeringControlWrenchAllocationObject(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  Eigen::VectorXd lambda(x.size());
  for(int i = 0; i < x.size(); i++)
    {
      lambda(i) = x.at(i);
    }

  return lambda.norm();

}

double steeringControlWrenchAllocationConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::VectorXd target_wrench_acc_cog = controller->getTargetWrenchAccCog();
  Eigen::MatrixXd full_q_mat = controller->getFullQ();

  // std::cout << full_q_mat << std::endl;
  Eigen::VectorXd lambda(x.size());
  for(int i = 0; i < x.size(); i++)
    {
      lambda(i) = x.at(i);
    }

  double diff = (full_q_mat * lambda - target_wrench_acc_cog).norm();
  // double diff = 0.0;
  // std::cout << full_q_mat * lambda - target_wrench_acc_cog << std::endl;
  // std::cout << std::endl;
  // std::cout << (full_q_mat * lambda - target_wrench_acc_cog).norm() << std::endl;
  // std::cout << std::endl;

  return diff;
}

void RollingController::osqpPractice()
{
  int n_variables = 2 * motor_num_;
  int n_constraints = controlled_q_mat_.rows();

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

  Eigen::MatrixXd A(n_constraints, n_variables);
  A = controlled_q_mat_;
  // std::cout << A << std::endl;
  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();

  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(n_variables);

  double epsilon = 1e-4;
  Eigen::VectorXd lower_bound = target_wrench_acc_cog_ - Eigen::VectorXd::Ones(n_constraints) * epsilon;
  Eigen::VectorXd upper_bound = target_wrench_acc_cog_ + Eigen::VectorXd::Ones(n_constraints) * epsilon;

  OsqpEigen::Solver solver;

  solver.settings()->setVerbosity(true);
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

  for(int i = 0; i < n_variables; i++)
    {
      std::cout << solution(i) << " ";
    }
  std::cout << std::endl;

}

void RollingController::calcSteeringTargetLambda()
{
  int n_variables = 2 * motor_num_;
  int n_constraints = 3 + 1 + 2 + 2;
  double epsilon = 0.001;

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

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.topRows(3) = full_q_rot_;                                                    //    eq constraint about rpy torque
  A.block(3, 0, 1, n_variables) = full_q_mat_.row(Z);                            // in eq constraint about z
  A.block(4, 0, 1, n_variables) = full_q_mat_.row(X) - steering_mu_ * full_q_mat_.row(Z);  // in eq constraint about x
  A.block(5, 0, 1, n_variables) = full_q_mat_.row(X) + steering_mu_ * full_q_mat_.row(Z);  // in eq constraint about x
  A.block(6, 0, 1, n_variables) = full_q_mat_.row(Y) - steering_mu_ * full_q_mat_.row(Z);  // in eq constraint about y
  A.block(7, 0, 1, n_variables) = full_q_mat_.row(Y) + steering_mu_ * full_q_mat_.row(Z);  // in eq constraint about y

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  lower_bound
    <<
    target_wrench_acc_cog_(control_dof_ - 6 + ROLL) - epsilon,
    target_wrench_acc_cog_(control_dof_ - 6 + PITCH) - epsilon,
    target_wrench_acc_cog_(control_dof_ - 6 + YAW) - epsilon,
    steering_z_acc_min_,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound <<
    target_wrench_acc_cog_(control_dof_ - 6 + ROLL) + epsilon,
    target_wrench_acc_cog_(control_dof_ - 6 + PITCH) + epsilon,
    target_wrench_acc_cog_(control_dof_ - 6 + YAW) + epsilon,
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

  full_lambda_trans_ = solution;
  full_lambda_all_ = solution;

}
