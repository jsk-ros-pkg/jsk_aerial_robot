#include <benchmark/benchmark.h>
#include <aerial_robot_dynamics/robot_model.h>
#include <aerial_robot_dynamics/robot_model_ros.h>
#include <aerial_robot_dynamics/math_utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  aerial_robot_dynamics::PinocchioRobotModelRos pinocchio_robot_model_ros(nh);

  aerial_robot_dynamics::PinocchioRobotModel::Config config;
  config.verbose = false;  // print only the benchmark output
  aerial_robot_dynamics::PinocchioRobotModel pinocchio_robot_model(
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getRobotDescription(),
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getPinocchioRobotDescription(), true, config);

  const int DATA_SIZE = 4096;
  std::vector<Eigen::VectorXd> q_vec(DATA_SIZE);
  std::vector<Eigen::VectorXd> thrust_vec(DATA_SIZE);

  for (int i = 0; i < DATA_SIZE; i++)
  {
    q_vec[i] = pinocchio::randomConfiguration(*(pinocchio_robot_model.getModel()));
    thrust_vec[i] = Eigen::VectorXd::Ones(pinocchio_robot_model.getRotorNum());
    aerial_robot_dynamics::addNoise(thrust_vec[i], 0.1);
  }

  benchmark::RegisterBenchmark("ThrustGenforceDqHessianBenchmark", [&](benchmark::State& state) {
    size_t idx = 0;

    for (auto _ : state)
    {
      const auto& q = q_vec[idx & (DATA_SIZE - 1)];
      const auto& thrust = thrust_vec[idx & (DATA_SIZE - 1)];

      Eigen::MatrixXd tauext_by_thrust_q_derivative_hessian =
          pinocchio_robot_model.computeTauExtByThrustQDerivativeHessian(q, thrust);

      benchmark::DoNotOptimize(tauext_by_thrust_q_derivative_hessian);

      idx++;
    }
  });

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}
