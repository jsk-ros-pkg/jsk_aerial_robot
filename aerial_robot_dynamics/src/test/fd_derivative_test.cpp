#include <aerial_robot_dynamics/robot_model_test.h>
#include <aerial_robot_dynamics/math_utils.h>
#include <chrono>

using namespace aerial_robot_dynamics;

bool PinocchioRobotModelTest::forwardDynamicsDerivativesTest(bool verbose)
{
  Eigen::VectorXd q = pinocchio::randomConfiguration(*(robot_model_->getModel()));
  Eigen::VectorXd v = Eigen::VectorXd::Ones(robot_model_->getModel()->nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Ones(robot_model_->getModel()->nv);
  Eigen::VectorXd thrust = Eigen::VectorXd::Ones(robot_model_->getRotorNum());

  addNoise(v, 0.1);
  addNoise(tau, 0.1);
  addNoise(thrust, 0.1);

  // normal ABA Derivatives
  auto start = std::chrono::high_resolution_clock::now();
  pinocchio::computeABADerivatives(*(robot_model_->getModel()), *(robot_model_->getData()), q, v, tau);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "ABADrivatives time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms" << std::endl;

  Eigen::MatrixXd aba_partial_dq = robot_model_->getData()->ddq_dq;
  Eigen::MatrixXd aba_partial_dv = robot_model_->getData()->ddq_dv;
  Eigen::MatrixXd aba_partial_dtau = robot_model_->getData()->Minv;

  // ABA Derivatives with thrust
  start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXd aba_thrust_partial_dthrust = robot_model_->forwardDynamicsDerivatives(q, v, tau, thrust);
  end = std::chrono::high_resolution_clock::now();
  std::cout << "ABA with thrust Derivatives time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms" << std::endl;

  Eigen::MatrixXd aba_thrust_partial_dq = robot_model_->getData()->ddq_dq;
  Eigen::MatrixXd aba_thrust_partial_dv = robot_model_->getData()->ddq_dv;
  Eigen::MatrixXd aba_thrust_partial_dtau = robot_model_->getData()->Minv;

  if(verbose)
    {
      std::cout << "q: " << q.transpose() << std::endl;
      std::cout << "v: " << v.transpose() << std::endl;
      std::cout << "tau: " << tau.transpose() << std::endl;
      std::cout << "thrust: " << thrust.transpose() << std::endl;

      std::cout << "aba_partial_dq: " << std::endl;
      std::cout << aba_partial_dq << std::endl;
      std::cout << "aba_thrust_partial_dq: " << std::endl;
      std::cout << aba_thrust_partial_dq << std::endl;
      std::cout << "aba_partial_dv: " << std::endl;
      std::cout << aba_partial_dv << std::endl;
      std::cout << "aba_partial_dtau: " << std::endl;
      std::cout << aba_partial_dtau << std::endl;
      std::cout << "aba_thrust_partial_dthrust: " << std::endl;
      std::cout << aba_thrust_partial_dthrust << std::endl;

      std::cout << "aba_partial_dq is changed by thrust: ";
      std::cout << !(aba_partial_dq.isApprox(aba_thrust_partial_dq, 1e-6)) << std::endl;
      std::cout << "aba_partial_dv is changed by thrust: ";
      std::cout << !(aba_partial_dv.isApprox(aba_thrust_partial_dv, 1e-6)) << std::endl;
      std::cout << "aba_partial_dtau is changed by thrust: ";
      std::cout << !(aba_partial_dtau.isApprox(aba_thrust_partial_dtau, 1e-6)) << std::endl;
    }

  // compare with numerical derivative
  double epsilon = 1e-6;
  Eigen::VectorXd original_thrust = thrust;
  Eigen::VectorXd original_a = robot_model_->forwardDynamics(q, v, tau, original_thrust);
  Eigen::MatrixXd aba_thrust_partial_dthrust_num = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv, robot_model_->getRotorNum());
  for(int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      thrust = original_thrust;
      thrust(i) += epsilon;
      Eigen::VectorXd a_plus = robot_model_->forwardDynamics(q, v, tau, thrust);
      aba_thrust_partial_dthrust_num.col(i) = (a_plus - original_a) / epsilon;
    }

  if(verbose)
    {
      std::cout << "aba_thrust_partial_dthrust numerical: " << std::endl;
      std::cout << aba_thrust_partial_dthrust_num << std::endl;
    }

  return (aba_thrust_partial_dthrust.isApprox(aba_thrust_partial_dthrust_num, 1e-6));
}
