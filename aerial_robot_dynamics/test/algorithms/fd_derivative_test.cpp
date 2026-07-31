#include <aerial_robot_dynamics/test_utils.h>
#include <aerial_robot_dynamics/math_utils.h>

#include <gtest/gtest.h>

#include <chrono>
#include <string>

using namespace aerial_robot_dynamics;

TEST(ForwardDynamicsDerivatives, MatchNumericalDerivatives)
{
  PinocchioRobotModel& robot_model = getTestRobotModel();
  const bool verbose = testVerbose();

  for (int trial = 0; trial < testTrials(); trial++)
  {
    SCOPED_TRACE("trial " + std::to_string(trial));

    Eigen::VectorXd q = pinocchio::randomConfiguration(*(robot_model.getModel()));
    Eigen::VectorXd v = Eigen::VectorXd::Ones(robot_model.getModel()->nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Ones(robot_model.getModel()->nv);
    Eigen::VectorXd thrust = Eigen::VectorXd::Ones(robot_model.getRotorNum());

    addNoise(v, 0.1);
    addNoise(tau, 0.1);
    addNoise(thrust, 0.1);

    // normal ABA Derivatives
    auto start = std::chrono::high_resolution_clock::now();
    pinocchio::computeABADerivatives(*(robot_model.getModel()), *(robot_model.getData()), q, v, tau);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "ABADrivatives time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;

    Eigen::MatrixXd aba_partial_dq = robot_model.getData()->ddq_dq;
    Eigen::MatrixXd aba_partial_dv = robot_model.getData()->ddq_dv;
    Eigen::MatrixXd aba_partial_dtau = robot_model.getData()->Minv;

    // ABA Derivatives with thrust
    start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd aba_thrust_partial_dthrust = robot_model.forwardDynamicsDerivatives(q, v, tau, thrust);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "ABA with thrust Derivatives time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;

    Eigen::MatrixXd aba_thrust_partial_dq = robot_model.getData()->ddq_dq;
    Eigen::MatrixXd aba_thrust_partial_dv = robot_model.getData()->ddq_dv;
    Eigen::MatrixXd aba_thrust_partial_dtau = robot_model.getData()->Minv;

    if (verbose)
    {
      std::cout << "q: " << q.transpose() << std::endl;
      std::cout << "v: " << v.transpose() << std::endl;
      std::cout << "tau: " << tau.transpose() << std::endl;
      std::cout << "thrust: " << thrust.transpose() << std::endl;
    }

    // compare with numerical derivative
    double epsilon = 1e-6;
    Eigen::VectorXd original_q = q;
    Eigen::VectorXd original_thrust = thrust;
    Eigen::VectorXd original_a = robot_model.forwardDynamics(original_q, v, tau, original_thrust);

    // check partial_dq
    Eigen::MatrixXd aba_thrust_partial_dq_num =
        Eigen::MatrixXd::Zero(robot_model.getModel()->nv, robot_model.getModel()->nv);
    for (int i = 0; i < robot_model.getModel()->nv; i++)
    {
      q = original_q;
      Eigen::VectorXd delta_v = Eigen::VectorXd::Zero(robot_model.getModel()->nv);
      delta_v(i) = 1.0;
      q = pinocchio::integrate(*(robot_model.getModel()), original_q, delta_v * epsilon);
      Eigen::VectorXd a_plus = robot_model.forwardDynamics(q, v, tau, original_thrust);
      aba_thrust_partial_dq_num.col(i) = (a_plus - original_a) / epsilon;
    }

    // check partial_dthrust
    Eigen::MatrixXd aba_thrust_partial_dthrust_num =
        Eigen::MatrixXd::Zero(robot_model.getModel()->nv, robot_model.getRotorNum());
    for (int i = 0; i < robot_model.getRotorNum(); i++)
    {
      thrust = original_thrust;
      thrust(i) += epsilon;
      Eigen::VectorXd a_plus = robot_model.forwardDynamics(original_q, v, tau, thrust);
      aba_thrust_partial_dthrust_num.col(i) = (a_plus - original_a) / epsilon;
    }

    if (verbose)
    {
      std::cout << "aba_partial_dq is changed by thrust: ";
      std::cout << !(aba_partial_dq.isApprox(aba_thrust_partial_dq, 1e-6)) << std::endl;
      if (!(aba_partial_dq.isApprox(aba_thrust_partial_dq, 1e-6)))
      {
        std::cout << "aba_partial_dq: " << std::endl;
        std::cout << aba_partial_dq << std::endl;
        std::cout << "aba_thrust_partial_dq: " << std::endl;
        std::cout << aba_thrust_partial_dq << std::endl;
      }
      else
      {
        std::cout << "aba_partial_dq: " << std::endl;
        std::cout << aba_partial_dq << std::endl;
      }
      std::cout << "aba_thrust_partial_dq numerical: " << std::endl;
      std::cout << aba_thrust_partial_dq_num << std::endl;
      std::cout << std::endl;

      std::cout << "aba_partial_dv is changed by thrust: ";
      std::cout << !(aba_partial_dv.isApprox(aba_thrust_partial_dv, 1e-6)) << std::endl;
      if (!(aba_partial_dv.isApprox(aba_thrust_partial_dv, 1e-6)))
      {
        std::cout << "aba_partial_dv: " << std::endl;
        std::cout << aba_partial_dv << std::endl;
        std::cout << "aba_thrust_partial_dv: " << std::endl;
        std::cout << aba_thrust_partial_dv << std::endl;
      }
      else
      {
        std::cout << "aba_partial_dv: " << std::endl;
        std::cout << aba_partial_dv << std::endl;
      }
      std::cout << std::endl;

      std::cout << "aba_partial_dtau is changed by thrust: ";
      std::cout << !(aba_partial_dtau.isApprox(aba_thrust_partial_dtau, 1e-6)) << std::endl;
      if (!(aba_partial_dtau.isApprox(aba_thrust_partial_dtau, 1e-6)))
      {
        std::cout << "aba_partial_dtau: " << std::endl;
        std::cout << aba_partial_dtau << std::endl;
        std::cout << "aba_thrust_partial_dtau: " << std::endl;
        std::cout << aba_thrust_partial_dtau << std::endl;
      }
      else
      {
        std::cout << "aba_partial_dtau: " << std::endl;
        std::cout << aba_partial_dtau << std::endl;
      }
      std::cout << std::endl;

      std::cout << "aba_thrust_partial_dthrust: " << std::endl;
      std::cout << aba_thrust_partial_dthrust << std::endl;
      std::cout << "aba_thrust_partial_dthrust numerical: " << std::endl;
      std::cout << aba_thrust_partial_dthrust_num << std::endl;
    }

    EXPECT_TRUE(aba_thrust_partial_dq.isApprox(aba_thrust_partial_dq_num, 1e-4))
        << "max error: " << (aba_thrust_partial_dq - aba_thrust_partial_dq_num).cwiseAbs().maxCoeff();
    EXPECT_TRUE(aba_thrust_partial_dthrust.isApprox(aba_thrust_partial_dthrust_num, 1e-4))
        << "max error: " << (aba_thrust_partial_dthrust - aba_thrust_partial_dthrust_num).cwiseAbs().maxCoeff();
  }
}
