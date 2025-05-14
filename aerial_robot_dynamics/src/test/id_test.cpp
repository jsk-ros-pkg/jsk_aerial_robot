#include <aerial_robot_dynamics/robot_model_test.h>
#include <aerial_robot_dynamics/math_utils.h>
#include <chrono>

using namespace aerial_robot_dynamics;

bool PinocchioRobotModelTest::inverseDynamicsTest(bool verbose)
{
  Eigen::VectorXd q = robot_model_->getResetConfiguration();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model_->getModel()->nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(robot_model_->getModel()->nv);

  addNoise(v, 0.1);
  addNoise(a, 0.1);

  auto start = std::chrono::high_resolution_clock::now();
  Eigen::VectorXd tau = pinocchio::rnea(*(robot_model_->getModel()), *(robot_model_->getData()), q, v, a);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "RNEA time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0
            << " ms" << std::endl;

  start = std::chrono::high_resolution_clock::now();
  Eigen::VectorXd tau_thrust;
  bool ok = robot_model_->inverseDynamics(q, v, a, tau_thrust);
  end = std::chrono::high_resolution_clock::now();
  std::cout << "ID " << (ok ? "solved. " : "not solved. ")
            << "time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
            << std::endl;

  if (verbose)
  {
    std::cout << "q: " << std::endl;
    std::cout << q.transpose() << std::endl;
    std::cout << "v: " << std::endl;
    std::cout << v.transpose() << std::endl;
    std::cout << "a: " << std::endl;
    std::cout << a.transpose() << std::endl;
    std::cout << "tau: " << std::endl;
    std::cout << tau.transpose() << std::endl;
    std::cout << "tau_thrust: " << std::endl;
    std::cout << tau_thrust.transpose() << std::endl;
  }

  // check with result of fd
  Eigen::VectorXd thrust = tau_thrust.tail(robot_model_->getRotorNum());

  Eigen::VectorXd a_fd = robot_model_->forwardDynamics(q, v, tau_thrust.head(robot_model_->getModel()->nv), thrust);

  if (verbose)
  {
    std::cout << "a_fd: " << std::endl;
    std::cout << a_fd.transpose() << std::endl;
    std::cout << "error norm: " << (a - a_fd).norm() << std::endl;
  }

  return ((a - a_fd).array().abs() < 1e-4).all();
}
