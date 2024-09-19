#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

namespace aerial_robot_model {
  /* psuedo inverse */
  /* https://gist.github.com/javidcf/25066cf85e71105d57b6 */
  template <class MatT>
  Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
  pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
  {
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
      if (singularValues(i) > tolerance)
        {
          singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
      else
        {
          singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
  }

  inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec)
  {
    Eigen::Matrix3d skew_mat;
    skew_mat << 0.0, -vec(2), vec(1),
      vec(2), 0.0, -vec(0),
      -vec(1), vec(0), 0.0;
    return skew_mat;
  }

  inline double reluApprox(double x, double epsilon = 10)
  {
    return std::log(1 + std::exp(x * epsilon)) / epsilon;
  }

  // differential of reluApprox
  inline double sigmoid(double x, double epsilon = 10)
  {
    return 1 / (1 + std::exp(- x * epsilon));
  }

  inline double absApprox(double x, double epsilon = 10)
  {
    return std::log(std::exp(- x * epsilon) + std::exp(x * epsilon)) / epsilon;
  }

  // differential of absApprox
  inline double tanh(double x, double epsilon = 10)
  {
    double a = std::exp(-x * epsilon);
    double b = std::exp(x * epsilon);
    return (b - a) / (b + a);
  }


} //namespace aerial_robot_model
