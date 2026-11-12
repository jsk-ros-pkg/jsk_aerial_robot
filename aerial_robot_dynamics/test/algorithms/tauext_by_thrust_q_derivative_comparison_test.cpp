#include <aerial_robot_dynamics/test_utils.h>
#include <aerial_robot_dynamics/math_utils.h>

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

using namespace aerial_robot_dynamics;

TEST(TauExtByThrustQDerivative, ThrustGenforceDqRneaHessianComparison)
{
  PinocchioRobotModel& robot_model = getTestRobotModel();
  const bool verbose = testVerbose();

  for (int trial = 0; trial < testTrials(); trial++)
  {
    SCOPED_TRACE("trial " + std::to_string(trial));

    Eigen::VectorXd q = pinocchio::randomConfiguration(*(robot_model.getModel()));
    Eigen::VectorXd thrust = Eigen::VectorXd::Ones(robot_model.getRotorNum());

    addNoise(thrust, 0.1);

    int nv = robot_model.getModel()->nv;
    Eigen::MatrixXd tauext_by_thrust_q_derivative_hessian = Eigen::MatrixXd::Zero(nv, nv);
    Eigen::MatrixXd tauext_by_thrust_q_derivative_rnea = Eigen::MatrixXd::Zero(nv, nv);

    // hessian based method
    {
      auto start = std::chrono::high_resolution_clock::now();
      tauext_by_thrust_q_derivative_hessian = robot_model.computeTauExtByThrustQDerivativeHessian(q, thrust);
      auto end = std::chrono::high_resolution_clock::now();

      std::cout << "TauExt by Thrust Q Derivative Hessian time: "
                << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
                << std::endl;

      if (verbose)
      {
        std::cout << "tauext_by_thrust_q_derivative_hessian" << std::endl;
        std::cout << tauext_by_thrust_q_derivative_hessian << std::endl;
      }
    }

    // rnea method
    {
      auto start = std::chrono::high_resolution_clock::now();
      tauext_by_thrust_q_derivative_rnea = robot_model.computeTauExtByThrustQDerivative(q, thrust);
      auto end = std::chrono::high_resolution_clock::now();

      std::cout << "TauExt by Thrust Q Derivative RNEA time: "
                << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
                << std::endl;

      if (verbose)
      {
        std::cout << "tauext_by_thrust_q_derivative_rnea" << std::endl;
        std::cout << tauext_by_thrust_q_derivative_rnea << std::endl;
      }
    }

    EXPECT_TRUE(tauext_by_thrust_q_derivative_hessian.isApprox(tauext_by_thrust_q_derivative_rnea, 1e-4))
        << "max error: "
        << (tauext_by_thrust_q_derivative_hessian - tauext_by_thrust_q_derivative_rnea).cwiseAbs().maxCoeff();
  }
}
