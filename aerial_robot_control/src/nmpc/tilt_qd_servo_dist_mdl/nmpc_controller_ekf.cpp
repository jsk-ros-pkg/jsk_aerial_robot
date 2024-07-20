//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller_ekf.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwEKF::initParams()
{
  TiltQdServoNMPC::initParams();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle ekf_nh(control_nh, "ekf");

  sim_solver_ptr_->initialize();  // TODO: try to put it together with mpc_solver_ptr_->initialize() in the future

  /* EKF */
  double Qp, Qv, Qq, Qw, Qa, Qd_f_i, Qd_tau_b;
  getParam<double>(ekf_nh, "Qp", Qp, 0.01);
  getParam<double>(ekf_nh, "Qv", Qv, 0.01);
  getParam<double>(ekf_nh, "Qq", Qq, 0.01);
  getParam<double>(ekf_nh, "Qw", Qw, 0.01);
  getParam<double>(ekf_nh, "Qa", Qa, 0.01);
  getParam<double>(ekf_nh, "Qd_f_i", Qd_f_i, 0.5);
  getParam<double>(ekf_nh, "Qd_tau_b", Qd_tau_b, 0.1);

  double Rp, Rv, Rq, Rw, Ra;
  getParam<double>(ekf_nh, "Rp", Rp, 0.005);
  getParam<double>(ekf_nh, "Rv", Rv, 0.05);
  getParam<double>(ekf_nh, "Rq", Rq, 0.01);
  getParam<double>(ekf_nh, "Rw", Rw, 0.05);
  getParam<double>(ekf_nh, "Ra", Ra, 0.01);

  int& NX = sim_solver_ptr_->NX_;
  int& NU = sim_solver_ptr_->NU_;

  Eigen::MatrixXd A = KalmanFilter::stdVec2EigenMat(sim_solver_ptr_->getMatrixA(), NX, NX);

  Eigen::MatrixXd B = KalmanFilter::stdVec2EigenMat(sim_solver_ptr_->getMatrixB(), NX, NU);

  Eigen::MatrixXd G = Eigen::MatrixXd ::Identity(NX, NX);

  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(17, NX);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(NX, NX);
  Q(0, 0) = Qp, Q(1, 1) = Qp, Q(2, 2) = Qp;
  Q(3, 3) = Qv, Q(4, 4) = Qv, Q(5, 5) = Qv;
  Q(6, 6) = Qq, Q(7, 7) = Qq, Q(8, 8) = Qq, Q(9, 9) = Qq;
  Q(10, 10) = Qw, Q(11, 11) = Qw, Q(12, 12) = Qw;
  Q(13, 13) = Qa, Q(14, 14) = Qa, Q(15, 15) = Qa, Q(16, 16) = Qa;
  Q(17, 17) = Qd_f_i, Q(18, 18) = Qd_f_i, Q(19, 19) = Qd_f_i;
  Q(20, 20) = Qd_tau_b, Q(21, 21) = Qd_tau_b, Q(22, 22) = Qd_tau_b;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(17, 17);
  R(0, 0) = Rp, R(1, 1) = Rp, R(2, 2) = Rp;
  R(3, 3) = Rv, R(4, 4) = Rv, R(5, 5) = Rv;
  R(6, 6) = Rq, R(7, 7) = Rq, R(8, 8) = Rq, R(9, 9) = Rq;
  R(10, 10) = Rw, R(11, 11) = Rw, R(12, 12) = Rw;
  R(13, 13) = Ra, R(14, 14) = Ra, R(15, 15) = Ra, R(16, 16) = Ra;

  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(NX, NX);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(NX);
  x(6) = 1.0; // q.w

  ekf_.init(A, B, G, C, Q, R, P, x);
}

std::vector<double> nmpc::TiltQdServoNMPCwEKF::meas2VecX()
{
  Eigen::VectorXd z = Eigen::VectorXd::Zero(17);
  z(0) = odom_.pose.pose.position.x;
  z(1) = odom_.pose.pose.position.y;
  z(2) = odom_.pose.pose.position.z;
  z(3) = odom_.twist.twist.linear.x;
  z(4) = odom_.twist.twist.linear.y;
  z(5) = odom_.twist.twist.linear.z;
  z(6) = odom_.pose.pose.orientation.w;
  z(7) = odom_.pose.pose.orientation.x;
  z(8) = odom_.pose.pose.orientation.y;
  z(9) = odom_.pose.pose.orientation.z;
  z(10) = odom_.twist.twist.angular.x;
  z(11) = odom_.twist.twist.angular.y;
  z(12) = odom_.twist.twist.angular.z;
  z(13) = joint_angles_.at(0);
  z(14) = joint_angles_.at(1);
  z(15) = joint_angles_.at(2);
  z(16) = joint_angles_.at(3);
  ekf_.update(z);

  calcDisturbWrench();

  // get bx0
  vector<double> bx0(mpc_solver_ptr_->NBX0_, 0);
  for (int i = 0; i < mpc_solver_ptr_->NBX0_; i++)
    bx0[i] = ekf_.getX(i);

  // time update  TODO: optimize the for loop; put this part after mpc_solver_ptr_->solve() in the future
  Eigen::VectorXd x_eigen = ekf_.getX();
  std::vector<double> x(sim_solver_ptr_->NX_, 0.0);
  for (int i = 0; i < sim_solver_ptr_->NX_; i++)
    x[i] = x_eigen(i);

  std::vector<double> u(sim_solver_ptr_->NU_, 0.0);
  for (int i = 0; i < sim_solver_ptr_->NU_; i++)
    u[i] = getCommand(i);

  sim_solver_ptr_->solve(x, u);

  int& NX = sim_solver_ptr_->NX_;
  Eigen::VectorXd xo = KalmanFilter::stdVec2EigenVec(sim_solver_ptr_->xo_);
  Eigen::MatrixXd A = KalmanFilter::stdVec2EigenMat(sim_solver_ptr_->getMatrixA(), NX, NX);
  ekf_.predictWithX(xo, A);

  return bx0;
}

void nmpc::TiltQdServoNMPCwEKF::calcDisturbWrench()
{
  dist_force_w_.x = ekf_.getX(17);
  dist_force_w_.y = ekf_.getX(18);
  dist_force_w_.z = ekf_.getX(19);
  dist_torque_cog_.x = ekf_.getX(20);
  dist_torque_cog_.y = ekf_.getX(21);
  dist_torque_cog_.z = ekf_.getX(22);
}

void nmpc::TiltQdServoNMPCwEKF::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  TiltQdServoNMPC::cfgNMPCCallback(config, level);
  // TODO： implement the callback function
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoNMPCwEKF, aerial_robot_control::ControlBase);
