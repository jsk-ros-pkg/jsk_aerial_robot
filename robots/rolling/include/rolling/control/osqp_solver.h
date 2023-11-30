#pragma once

#include <rolling/control/rolling_controller.h>
#include <osqp_slsqp/slsqp.h>

class slsqpSolver
{
private:
  int n_variables_;
  int n_constraints_;
  Eigen::VectorXd target_wrench_;
  Eigen::MatrixXd full_q_;
  Eigen::VectorXd initial_x_;
  double mass_;
  double gravity_;
  double mu_;

  Eigen::VectorXd solution_;

public:
  slsqpSolver(int n_variables, int n_constraints,
              Eigen::VectorXd target_wrench, Eigen::MatrixXd full_q,
              Eigen::VectorXd initial_x,
              double mass, double gravity, double mu)
  {
    n_variables_ = n_variables;
    n_constraints_ = n_constraints;
    target_wrench_ = target_wrench;
    full_q_ = full_q;
    initial_x_ = initial_x;
    mass_ = mass;
    gravity_ = gravity;
    mu_ = mu;
  }

  void solve()
  {
    auto cost_function_gradient = [this](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
      Eigen::VectorXd grad(this->n_variables_);
      for(int i = 0; i < this->n_variables_; i++) grad(i) = 2.0 * x(i);
      return grad;
    };

    auto inequality_constraint = [this](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
      Eigen::VectorXd constraint(this->n_constraints_);

      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(this->n_constraints_, this->n_variables_);
      Eigen::MatrixXd full_q_trans_force = this->full_q_.topRows(3) * mass_;
      A.topRows(3) = this->full_q_.bottomRows(3);
      A.block(3, 0, 1, this->n_variables_) = full_q_trans_force.row(2);
      A.block(4, 0, 1, this->n_variables_) = full_q_trans_force.row(0) - this->mu_ * full_q_trans_force.row(2);
      A.block(5, 0, 1, this->n_variables_) = full_q_trans_force.row(0) + this->mu_ * full_q_trans_force.row(2);
      A.block(6, 0, 1, this->n_variables_) = full_q_trans_force.row(1) - this->mu_ * full_q_trans_force.row(2);
      A.block(7, 0, 1, this->n_variables_) = full_q_trans_force.row(1) + this->mu_ * full_q_trans_force.row(2);
      constraint = A * x;
      return constraint;
    };

    auto inequality_constraint_gradient = [this](const Eigen::VectorXd& x) -> Eigen::MatrixXd
    {
      Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(this->n_constraints_, this->n_variables_);
      Eigen::MatrixXd full_q_trans_force = this->full_q_.topRows(3) * mass_;
      grad.topRows(3) = this->full_q_.bottomRows(3);
      grad.block(3, 0, 1, this->n_variables_) = full_q_trans_force.row(2);
      grad.block(4, 0, 1, this->n_variables_) = full_q_trans_force.row(0) - this->mu_ * full_q_trans_force.row(2);
      grad.block(5, 0, 1, this->n_variables_) = full_q_trans_force.row(0) + this->mu_ * full_q_trans_force.row(2);
      grad.block(6, 0, 1, this->n_variables_) = full_q_trans_force.row(1) - this->mu_ * full_q_trans_force.row(2);
      grad.block(7, 0, 1, this->n_variables_) = full_q_trans_force.row(1) + this->mu_ * full_q_trans_force.row(2);
      return grad;
    };

    Eigen::VectorXd x = initial_x_;

    Eigen::VectorXd upper_bound(n_constraints_);
    Eigen::VectorXd lower_bound(n_constraints_);

    lower_bound <<
      target_wrench_(3) - 1e-4,
      target_wrench_(4) - 1e-4,
      target_wrench_(5) - 1e-4,
      -OsqpEigen::INFTY,
      -mu_ * mass_ * gravity_,
      -OsqpEigen::INFTY,
      -mu_ * mass_ * gravity_,
      -OsqpEigen::INFTY;

    upper_bound <<
      target_wrench_(3) + 1e-4,
      target_wrench_(4) + 1e-4,
      target_wrench_(5) + 1e-4,
      mass_ * gravity_,
      OsqpEigen::INFTY,
      mu_ * mass_ * gravity_,
      OsqpEigen::INFTY,
      mu_ * mass_ * gravity_;

    int n_variables = 6;
    int n_constraints = 8;
    auto solver = makeSLSQPSolver(n_variables, n_constraints, cost_function_gradient, inequality_constraint, inequality_constraint_gradient, lower_bound, upper_bound, x, 1e-4, 1e-3);

    solver.solve();

    solution_ = solver.getLastX();
  }

  Eigen::VectorXd getSolution()
  {
    return solution_;
  }
};
