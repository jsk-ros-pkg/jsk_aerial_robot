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
