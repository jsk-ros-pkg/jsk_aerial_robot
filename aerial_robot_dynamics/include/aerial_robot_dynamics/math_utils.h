#pragma once

#include <Eigen/Dense>
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

inline Eigen::MatrixXd tensorContraction(const std::vector<Eigen::MatrixXd>& tensor1, const Eigen::VectorXd& vector1)
{
  for (int i = 0; i < tensor1.size(); i++)
  {
    if (tensor1.at(i).cols() != vector1.size())
    {
      throw std::invalid_argument("Tensor and vector dimensions do not match.");
    }
  }

  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(tensor1.at(0).rows(), tensor1.size());
  for (int i = 0; i < tensor1.size(); i++)
  {
    ret.col(i) = tensor1.at(i) * vector1;
  }
  return ret;
}
}  // namespace aerial_robot_dynamics
