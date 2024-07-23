//
// Created by li-jinjie on 24-7-18.
//

#ifndef AERIAL_ROBOT_CONTROL_EKF_ESTIMATOR_H
#define AERIAL_ROBOT_CONTROL_EKF_ESTIMATOR_H

#include "Eigen/Dense"

class EKFEstimator
{
public:
  EKFEstimator() = default;
  ~EKFEstimator() = default;

  void init(const double ts, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R_imu, const Eigen::MatrixXd& R_mocap,
            const Eigen::MatrixXd& R_servo, const Eigen::MatrixXd& P_init, const Eigen::VectorXd& x_init)
  {
    ts_ = ts, P_ = P_init, x_ = x_init;
    Q_ = Q, R_imu_ = R_imu, R_mocap_ = R_mocap, R_servo_ = R_servo;

    G_ = Eigen::MatrixXd::Zero(23, 6);
    for (int i = 0; i < 6; ++i)
      G_(17 + i, i) = 1.0;
    G_ *= ts_;

    C_mocap_ = Eigen::MatrixXd::Zero(7, 23);
    for (int i = 0; i < 3; ++i)
      C_mocap_(i, i) = 1.0;
    for (int i = 0; i < 4; ++i)
      C_mocap_(3 + i, 6 + i) = 1.0;

    C_servo_ = Eigen::MatrixXd::Zero(4, 23);
    for (int i = 0; i < 4; ++i)
      C_servo_(i, 13 + i) = 1.0;
  }

  /**
   * If use other nonlinear time update function to calculate x, use this method. The x and A are calculated outside.
   * @param x_new
   * @param A
   * @param G
   */
  void predict(const Eigen::VectorXd& x_new, const Eigen::MatrixXd& A)
  {
    x_ = x_new;
    P_ = A * P_ * A.transpose() + G_ * Q_ * G_.transpose();
  }

  void updateIMU(const Eigen::VectorXd& z, const Eigen::MatrixXd& C_imu, const Eigen::VectorXd& z_est)
  {
    Eigen::MatrixXd K = P_ * C_imu.transpose() * ((C_imu * P_ * C_imu.transpose() + R_imu_).inverse());
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(K.rows(), C_imu.cols()) - K * C_imu;
    P_ = tmp * P_ * tmp.transpose() + K * R_imu_ * K.transpose();
    x_ = x_ + K * (z - z_est);  // nonlinear observation model
  }

  void updateMoCap(const Eigen::VectorXd& z)
  {
    Eigen::MatrixXd K = P_ * C_mocap_.transpose() * ((C_mocap_ * P_ * C_mocap_.transpose() + R_mocap_).inverse());
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(K.rows(), C_mocap_.cols()) - K * C_mocap_;
    P_ = tmp * P_ * tmp.transpose() + K * R_mocap_ * K.transpose();
    x_ = x_ + K * (z - C_mocap_ * x_);
  }

  void updateServo(const Eigen::VectorXd& z)
  {
    Eigen::MatrixXd K = P_ * C_servo_.transpose() * ((C_servo_ * P_ * C_servo_.transpose() + R_servo_).inverse());
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(K.rows(), C_servo_.cols()) - K * C_servo_;
    P_ = tmp * P_ * tmp.transpose() + K * R_servo_ * K.transpose();
    x_ = x_ + K * (z - C_servo_ * x_);
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

  static std::vector<double> eigenVec2StdVec(const Eigen::VectorXd& vec)
  {
    std::vector<double> std_vec(vec.size());
    // Use std::memcpy to copy the data directly
    std::memcpy(std_vec.data(), vec.data(), vec.size() * sizeof(double));
    return std_vec;
  }

  /* clang-format off */
  // getter
  Eigen::VectorXd getX() const { return x_; }
  double getX(int i) const { return x_(i); }
  Eigen::MatrixXd getP() const { return P_; }
  Eigen::MatrixXd getG() const { return G_; }
  Eigen::MatrixXd getQ() const { return Q_; }
  Eigen::MatrixXd getCMocap() const { return C_mocap_; }
  Eigen::MatrixXd getCServo() const { return C_servo_; }
  Eigen::MatrixXd getRImu() const { return R_imu_; }
  Eigen::MatrixXd getRMocap() const { return R_mocap_; }
  Eigen::MatrixXd getRServo() const { return R_servo_; }

  // setter
  void setG(const Eigen::MatrixXd& G) { G_ = G; }
  void setQ(const Eigen::MatrixXd& Q) { Q_ = Q; }
  void setCMocap(const Eigen::MatrixXd& C_mocap) { C_mocap_ = C_mocap; }
  void setCServo(const Eigen::MatrixXd& C_servo) { C_servo_ = C_servo; }
  void setRImu(const Eigen::MatrixXd& R_imu) { R_imu_ = R_imu; }
  void setRMocap(const Eigen::MatrixXd& R_mocap) { R_mocap_ = R_mocap; }
  void setRServo(const Eigen::MatrixXd& R_servo) { R_servo_ = R_servo; }
  /* clang-format on */

private:
  double ts_ = 0;

  Eigen::MatrixXd P_;  // covariance matrix
  Eigen::VectorXd x_;  // state vector

  Eigen::MatrixXd G_;
  Eigen::MatrixXd Q_;  // process noise covariance matrix

  //  Eigen::MatrixXd C_imu_;    // IMU measurement matrix
  Eigen::MatrixXd C_mocap_;  // mocap measurement matrix
  Eigen::MatrixXd C_servo_;  // servo measurement matrix

  Eigen::MatrixXd R_imu_;    // IMU measurement noise covariance matrix
  Eigen::MatrixXd R_mocap_;  // mocap measurement noise covariance matrix
  Eigen::MatrixXd R_servo_;  // servo measurement noise covariance matrix
};

#endif  // AERIAL_ROBOT_CONTROL_EKF_ESTIMATOR_H
