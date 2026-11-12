#include <aerial_robot_dynamics/test_utils.h>
#include <aerial_robot_dynamics/math_utils.h>

#include <gtest/gtest.h>

#include <chrono>
#include <string>

using namespace aerial_robot_dynamics;

// DISABLED_ because OsqpEigen retains internal memory once its solver has run
// and leaks / crashes in its destructor at process teardown, which brings down
// the whole gtest binary.
// Run it explicitly with
// rostest aerial_robot_dynamics pinocchio_robot_model.test \
//   gtest_args:="--gtest_also_run_disabled_tests --gtest_filter=InverseDynamics.*"
TEST(InverseDynamics, DISABLED_InverseDynamicsOsqpTest)
{
  PinocchioRobotModel& robot_model = getTestRobotModel();
  const bool verbose = testVerbose();

  for (int trial = 0; trial < testTrials(); trial++)
  {
    SCOPED_TRACE("trial " + std::to_string(trial));

    Eigen::VectorXd q = robot_model.getResetConfiguration();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model.getModel()->nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(robot_model.getModel()->nv);

    addNoise(v, 0.1);
    addNoise(a, 0.1);

    auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd tau = pinocchio::rnea(*(robot_model.getModel()), *(robot_model.getData()), q, v, a);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "RNEA time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0
              << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd tau_thrust;
    bool solved = robot_model.inverseDynamics(q, v, a, tau_thrust);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "ID " << (solved ? "solved. " : "not solved. ")
              << "time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0
              << " ms" << std::endl;
    EXPECT_TRUE(solved);

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
    Eigen::VectorXd thrust = tau_thrust.tail(robot_model.getRotorNum());

    Eigen::VectorXd a_fd = robot_model.forwardDynamics(q, v, tau_thrust.head(robot_model.getModel()->nv), thrust);

    if (verbose)
    {
      std::cout << "a_fd: " << std::endl;
      std::cout << a_fd.transpose() << std::endl;
    }

    EXPECT_TRUE(((a - a_fd).array().abs() < 1e-4).all()) << "error norm: " << (a - a_fd).norm();
  }
}

TEST(InverseDynamics, InverseDynamicsProxqpTest)
{
  PinocchioRobotModel& robot_model = getTestRobotModel();
  const bool verbose = testVerbose();

  for (int trial = 0; trial < testTrials(); trial++)
  {
    SCOPED_TRACE("trial " + std::to_string(trial));

    Eigen::VectorXd q = robot_model.getResetConfiguration();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model.getModel()->nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(robot_model.getModel()->nv);

    addNoise(v, 0.1);
    addNoise(a, 0.1);

    auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd tau_thrust;
    bool solved = robot_model.inverseDynamicsProxqp(q, v, a, tau_thrust);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "ID (ProxQP) " << (solved ? "solved. " : "not solved. ")
              << "time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0
              << " ms" << std::endl;
    EXPECT_TRUE(solved);

    // check with result of fd
    Eigen::VectorXd thrust = tau_thrust.tail(robot_model.getRotorNum());

    Eigen::VectorXd a_fd = robot_model.forwardDynamics(q, v, tau_thrust.head(robot_model.getModel()->nv), thrust);

    if (verbose)
    {
      std::cout << "tau_thrust: " << tau_thrust.transpose() << std::endl;
      std::cout << "a_fd: " << a_fd.transpose() << std::endl;
    }

    EXPECT_TRUE(((a - a_fd).array().abs() < 1e-4).all()) << "error norm: " << (a - a_fd).norm();
  }
}
