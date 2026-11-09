#include <aerial_robot_dynamics/test_utils.h>
#include <aerial_robot_dynamics/math_utils.h>

#include <gtest/gtest.h>

#include <chrono>
#include <string>

using namespace aerial_robot_dynamics;

TEST(TauExtByThrustQDerivative, ThrustGenforceDqStaticTorqueTest)
{
  PinocchioRobotModel& robot_model = getTestRobotModel();
  const bool verbose = testVerbose();

  for (int trial = 0; trial < testTrials(); trial++)
  {
    SCOPED_TRACE("trial " + std::to_string(trial));

    Eigen::VectorXd q = pinocchio::randomConfiguration(*(robot_model.getModel()));
    Eigen::VectorXd thrust = Eigen::VectorXd::Ones(robot_model.getRotorNum());

    addNoise(thrust, 0.1);

    // Static torque based analytical derivative
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd tauext_by_thrust_q_derivative_ana =
        robot_model.computeTauExtByThrustQDerivativeStaticTorque(q, thrust);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "TauExt by Thrust Q Derivative Analytical time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;

    // Numerical derivative
    start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd tauext_by_thrust_q_derivative_num = robot_model.computeTauExtByThrustQDerivativeNum(q, thrust);
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

    EXPECT_TRUE(tauext_by_thrust_q_derivative_ana.isApprox(tauext_by_thrust_q_derivative_num, 1e-4))
        << "max error: "
        << (tauext_by_thrust_q_derivative_ana - tauext_by_thrust_q_derivative_num).cwiseAbs().maxCoeff();
  }
}
