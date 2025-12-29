#include <aerial_robot_dynamics/robot_model_test.h>
#include <aerial_robot_dynamics/math_utils.h>
#include <chrono>

using namespace aerial_robot_dynamics;

bool PinocchioRobotModelTest::computeTauExtByThrustQDerivativeTest(bool verbose)
{
  Eigen::VectorXd q = pinocchio::randomConfiguration(*(robot_model_->getModel()));
  Eigen::VectorXd thrust = Eigen::VectorXd::Ones(robot_model_->getRotorNum());

  addNoise(thrust, 0.1);

  auto start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXd tauext_by_thrust_q_derivative_ana = robot_model_->computeTauExtByThrustQDerivative(q, thrust);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "TauExt by Thrust Q Derivative Analytical time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
            << std::endl;

  start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXd tauext_by_thrust_q_derivative_num =
      Eigen::MatrixXd::Zero(robot_model_->getModel()->nv, robot_model_->getModel()->nv);

  double epsilon = 1e-6;
  Eigen::VectorXd original_q = q;
  Eigen::VectorXd original_tauext_by_thrust = robot_model_->computeTauExtByThrust(original_q, thrust);
  for (int i = 0; i < robot_model_->getModel()->nv; i++)
  {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model_->getModel()->nv);
    v(i) = 1.0;

    Eigen::VectorXd tmp_q = pinocchio::integrate(*(robot_model_->getModel()), original_q, v * epsilon);

    Eigen::VectorXd tauext_by_thrust_plus = robot_model_->computeTauExtByThrust(tmp_q, thrust);
    tauext_by_thrust_q_derivative_num.col(i) = (tauext_by_thrust_plus - original_tauext_by_thrust) / epsilon;
  }
  end = std::chrono::high_resolution_clock::now();
  std::cout << "TauExt by Thrust Q Derivative Numerical time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
            << std::endl;

  if (verbose)
  {
    std::cout << "tauext_by_thrust_q_derivative_ana" << std::endl;
    std::cout << tauext_by_thrust_q_derivative_ana << std::endl;
    std::cout << "tauext_by_thrust_q_derivative_num" << std::endl;
    std::cout << tauext_by_thrust_q_derivative_num << std::endl;
  }

  return tauext_by_thrust_q_derivative_ana.isApprox(tauext_by_thrust_q_derivative_num, 1e-4);
}
