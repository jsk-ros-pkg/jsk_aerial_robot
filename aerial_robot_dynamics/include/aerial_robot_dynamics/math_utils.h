#pragma once

#include <Eigen/Dense>
#include <random>
#include <vector>

namespace aerial_robot_dynamics
{
inline void addNoise(Eigen::VectorXd& vec, double sigma)
{
  std::default_random_engine generator(std::random_device{}());
  std::normal_distribution<double> distribution(0.0, sigma);
  for (int i = 0; i < vec.size(); ++i)
  {
    vec(i) += distribution(generator);
  }
}
}  // namespace aerial_robot_dynamics
