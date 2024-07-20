//
// Created by li-jinjie on 24-7-18.
//

#ifndef AERIAL_ROBOT_CONTROL_KALMAN_FILTER_H
#define AERIAL_ROBOT_CONTROL_KALMAN_FILTER_H

#include "Eigen/Dense"

class KalmanFilter
{
public:
  KalmanFilter() = default;
  ~KalmanFilter() = default;

  void init(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& G, const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P, const Eigen::VectorXd& x)
  {
    A_ = A, B_ = B, G_ = G, C_ = C, Q_ = Q, R_ = R, P_ = P, x_ = x;
  }

  void predict(const Eigen::VectorXd& u, const Eigen::MatrixXd& A = Eigen::MatrixXd::Zero(0, 0),
               const Eigen::MatrixXd& B = Eigen::MatrixXd::Zero(0, 0),
               const Eigen::MatrixXd& G = Eigen::MatrixXd::Zero(0, 0))
  {
    if (A.rows() != 0)
      A_ = A;
    if (B.rows() != 0)
      B_ = B;
    if (G.rows() != 0)
      G_ = G;

    x_ = A_ * x_ + B_ * u + G_ * Eigen::VectorXd::Random(G_.cols());
    P_ = A_ * P_ * A_.transpose() + G_ * Q_ * G_.transpose();
  }

  /**
   * If use other nonlinear time update function to calculate x, use this method. Otherwise, use void predict().
   * @param x_new
   * @param A
   * @param G
   */
  void predictWithX(const Eigen::VectorXd& x_new, const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& G = Eigen::MatrixXd::Zero(0, 0))
  {
    A_ = A;

    if (G.rows() != 0)
      G_ = G;

    x_ = x_new;

    P_ = A_ * P_ * A_.transpose() + G_ * Q_ * G_.transpose();
  }

  void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& C = Eigen::MatrixXd::Zero(0, 0))
  {
    if (C.rows() != 0)
      C_ = C;

    K_ = P_ * C_.transpose() * ((C_ * P_ * C_.transpose() + R_).inverse());
    x_ = x_ + K_ * (z - C_ * x_);  // TODO: consider using the nonlinear measurement model
    P_ = (Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K_ * C_) * P_;
  }

  static Eigen::MatrixXd stdVec2EigenMat(const std::vector<double>& vec, int rows, int cols)
  {
    if (vec.size() != rows * cols)
      throw std::length_error("The size of the vector does not match the size of the matrix");

    // Ensure the matrix uses row-major storage to match the vector's layout
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat(rows, cols);

    // Use std::memcpy to copy the data from the vector to the Eigen matrix
    std::memcpy(mat.data(), vec.data(), vec.size() * sizeof(double));

    return mat;
  }

  static Eigen::VectorXd stdVec2EigenVec(const std::vector<double>& vec)
  {
    Eigen::VectorXd eigen_vec(vec.size());
    // Use std::memcpy to copy the data directly
    std::memcpy(eigen_vec.data(), vec.data(), vec.size() * sizeof(double));
    return eigen_vec;
  }

  /* clang-format off */
  // getter
  Eigen::VectorXd getX() const { return x_; }
  double getX(int i) const { return x_(i); }
  Eigen::MatrixXd getP() const { return P_; }
  Eigen::MatrixXd getK() const { return K_; }

  // setter
  void setA(const Eigen::MatrixXd& A) { A_ = A; }
  void setB(const Eigen::MatrixXd& B) { B_ = B; }
  void setG(const Eigen::MatrixXd& G) { G_ = G; }
  void setC(const Eigen::MatrixXd& C) { C_ = C; }
  void setQ(const Eigen::MatrixXd& Q) { Q_ = Q; }
  void setR(const Eigen::MatrixXd& R) { R_ = R; }
  /* clang-format on */

private:
  Eigen::MatrixXd A_;  // state transition matrix
  Eigen::MatrixXd B_;  // control matrix
  Eigen::MatrixXd G_;  // disturbance matrix

  Eigen::MatrixXd C_;  // measurement matrix

  Eigen::MatrixXd Q_;  // process noise covariance matrix
  Eigen::MatrixXd R_;  // measurement noise covariance matrix

  Eigen::MatrixXd P_;  // covariance matrix
  Eigen::VectorXd x_;  // state vector
  Eigen::MatrixXd K_;  // Kalman gain
};

#endif  // AERIAL_ROBOT_CONTROL_KALMAN_FILTER_H
