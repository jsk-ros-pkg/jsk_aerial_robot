#include <aerial_robot_dynamics/test_utils.h>
#include <aerial_robot_dynamics/math_utils.h>

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

using namespace aerial_robot_dynamics;

TEST(TauExtByThrustDerivativeQDerivatives, MatchNumericalDerivatives)
{
  PinocchioRobotModel& robot_model = getTestRobotModel();
  const bool verbose = testVerbose();

  for (int trial = 0; trial < testTrials(); trial++)
  {
    SCOPED_TRACE("trial " + std::to_string(trial));

    Eigen::VectorXd q = robot_model.getResetConfiguration();

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q_ana =
        robot_model.computeTauExtByThrustDerivativeQDerivatives(q);  // compute analytical derivatives
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "TauExt by Thrust Derivative Q Derivatives Analytical time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;
    start = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q_num =
        robot_model.computeTauExtByThrustDerivativeQDerivativesNum(q);  // compute numerical derivatives
    end = std::chrono::high_resolution_clock::now();
    std::cout << "TauExt by Thrust Derivative Q Derivatives Numerical time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;

    if (verbose)
    {
      for (size_t i = 0; i < tauext_partial_thrust_partial_q_ana.size(); i++)
      {
        std::cout << "tauext_partial_thrust_partial_q_ana[" << i << "]" << std::endl;
        std::cout << tauext_partial_thrust_partial_q_ana.at(i) << std::endl;
        std::cout << "tauext_partial_thrust_partial_q_num[" << i << "]" << std::endl;
        std::cout << tauext_partial_thrust_partial_q_num.at(i) << std::endl;
      }
    }

    for (int i = 0; i < robot_model.getModel()->nv; i++)
    {
      EXPECT_LT(
          (tauext_partial_thrust_partial_q_ana.at(i) - tauext_partial_thrust_partial_q_num.at(i)).cwiseAbs().maxCoeff(),
          1e-4)
          << "tauext_partial_thrust_partial_q_ana[" << i << "] is not equal to tauext_partial_thrust_partial_q_num["
          << i << "]";
    }
  }
}
