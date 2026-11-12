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
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getPinocchioRobotDescription(), false, config);

  const int DATA_SIZE = 4096;
  std::vector<Eigen::VectorXd> q_vec(DATA_SIZE);
  std::vector<Eigen::VectorXd> v_vec(DATA_SIZE);
  std::vector<Eigen::VectorXd> a_vec(DATA_SIZE);

  for (int i = 0; i < DATA_SIZE; i++)
  {
    q_vec[i] = pinocchio_robot_model.getResetConfiguration();
    v_vec[i] = Eigen::VectorXd::Zero(pinocchio_robot_model.getModel()->nv);
    a_vec[i] = Eigen::VectorXd::Zero(pinocchio_robot_model.getModel()->nv);
    aerial_robot_dynamics::addNoise(v_vec[i], 0.1);
    aerial_robot_dynamics::addNoise(a_vec[i], 0.1);
  }

  benchmark::RegisterBenchmark("InverseDynamicsProxqpBenchmark", [&](benchmark::State& state) {
    size_t idx = 0;

    for (auto _ : state)
    {
      const auto& q = q_vec[idx & (DATA_SIZE - 1)];
      const auto& v = v_vec[idx & (DATA_SIZE - 1)];
      const auto& a = a_vec[idx & (DATA_SIZE - 1)];

      Eigen::VectorXd tau;
      bool solved = pinocchio_robot_model.inverseDynamicsProxqp(q, v, a, tau);

      benchmark::DoNotOptimize(solved);
      benchmark::DoNotOptimize(tau);

      idx++;
    }
  });

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}
