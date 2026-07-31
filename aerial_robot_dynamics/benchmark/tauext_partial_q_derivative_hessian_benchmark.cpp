#include <benchmark/benchmark.h>
#include <aerial_robot_dynamics/robot_model.h>
#include <aerial_robot_dynamics/robot_model_ros.h>
#include <aerial_robot_dynamics/math_utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  aerial_robot_dynamics::PinocchioRobotModelRos pinocchio_robot_model_ros(nh);

  aerial_robot_dynamics::PinocchioRobotModel pinocchio_robot_model(
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getRobotDescription(),
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getPinocchioRobotDescription(), true);

  const int DATA_SIZE = 4096;
  std::vector<Eigen::VectorXd> q_vec(DATA_SIZE);
  std::vector<Eigen::VectorXd> thrust_vec(DATA_SIZE);

  for (int i = 0; i < DATA_SIZE; i++)
  {
    q_vec[i] = pinocchio::randomConfiguration(*(pinocchio_robot_model.getModel()));
    thrust_vec[i] = Eigen::VectorXd::Ones(pinocchio_robot_model.getRotorNum());
    aerial_robot_dynamics::addNoise(thrust_vec[i], 0.1);
  }

  benchmark::RegisterBenchmark("BM_tauext_partial_q_hessian", [&](benchmark::State& state) {
    size_t idx = 0;

    for (auto _ : state)
    {
      const auto& q = q_vec[idx & (DATA_SIZE - 1)];
      const auto& thrust = thrust_vec[idx & (DATA_SIZE - 1)];

      Eigen::MatrixXd tauext_by_thrust_q_derivative_hessian =
          Eigen::MatrixXd::Zero(pinocchio_robot_model.getModel()->nv, pinocchio_robot_model.getModel()->nv);

      std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q =
          pinocchio_robot_model.computeTauExtByThrustDerivativeQDerivatives(q);

      for (int i = 0; i < pinocchio_robot_model.getModel()->nv; i++)
      {
        tauext_by_thrust_q_derivative_hessian.col(i) = tauext_partial_thrust_partial_q.at(i) * thrust;
      }

      benchmark::DoNotOptimize(tauext_by_thrust_q_derivative_hessian);

      idx++;
    }
  });

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}
