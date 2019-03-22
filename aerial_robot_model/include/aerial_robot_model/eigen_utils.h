#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

namespace aerial_robot_model {
  /* skew_symmetric matrix */
  const Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d vec)
  {
    Eigen::Matrix3d mat;
    mat << 0, -vec(2), vec(1),
      vec(2), 0, -vec(0),
      -vec(1), vec(0), 0;

    return mat;
  }

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
} //namespace aerial_robot_model
