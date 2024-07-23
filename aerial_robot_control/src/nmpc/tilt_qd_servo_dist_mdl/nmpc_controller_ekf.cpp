//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller_ekf.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwEKF::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  nmpc::TiltQdServoDistNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  sub_imu_ = nh_.subscribe("imu", 1, &TiltQdServoNMPCwEKF::callbackImu, this);
  sub_mocap_ = nh_.subscribe("mocap", 1, &TiltQdServoNMPCwEKF::callbackMoCap, this);
}

void nmpc::TiltQdServoNMPCwEKF::initParams()
{
  TiltQdServoNMPC::initParams();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle ekf_nh(control_nh, "ekf");

  sim_solver_ptr_->initialize();  // TODO: try to put it together with mpc_solver_ptr_->initialize() in the future

  /* EKF */
  double ekf_ts;
  getParam<double>(ekf_nh, "ts", ekf_ts, 0.01);

  double Qd_f_ixy, Qd_f_iz, Qd_tau_bxy, Qd_tau_bz;
  getParam<double>(ekf_nh, "Qd_f_ixy", Qd_f_ixy, 0.5);
  getParam<double>(ekf_nh, "Qd_f_iz", Qd_f_iz, 0.5);
  getParam<double>(ekf_nh, "Qd_tau_bxy", Qd_tau_bxy, 0.1);
  getParam<double>(ekf_nh, "Qd_tau_bz", Qd_tau_bz, 0.1);

  // IMU
  double R_imu_sf, R_imu_omega;
  getParam<double>(ekf_nh, "R_imu_sf", R_imu_sf, 0.01);
  getParam<double>(ekf_nh, "R_imu_omega", R_imu_omega, 0.1);

  // MoCap
  double R_mocap_p, R_mocap_q;
  getParam<double>(ekf_nh, "R_mocap_p", R_mocap_p, 0.005);
  getParam<double>(ekf_nh, "R_mocap_q", R_mocap_q, 0.01);

  // Servo
  double R_servo_a;
  getParam<double>(ekf_nh, "R_servo_a", R_servo_a, 0.01);

  int& NX = sim_solver_ptr_->NX_;
  int& NU = sim_solver_ptr_->NU_;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
  Q(0, 0) = Qd_f_ixy, Q(1, 1) = Qd_f_ixy, Q(2, 2) = Qd_f_iz;
  Q(3, 3) = Qd_tau_bxy, Q(4, 4) = Qd_tau_bxy, Q(5, 5) = Qd_tau_bz;

  Eigen::MatrixXd R_imu = Eigen::MatrixXd::Zero(6, 6);
  R_imu(0, 0) = R_imu_sf, R_imu(1, 1) = R_imu_sf, R_imu(2, 2) = R_imu_sf;
  R_imu(3, 3) = R_imu_omega, R_imu(4, 4) = R_imu_omega, R_imu(5, 5) = R_imu_omega;

  Eigen::MatrixXd R_mocap = Eigen::MatrixXd::Zero(7, 7);
  R_mocap(0, 0) = R_mocap_p, R_mocap(1, 1) = R_mocap_p, R_mocap(2, 2) = R_mocap_p;
  R_mocap(3, 3) = R_mocap_q, R_mocap(4, 4) = R_mocap_q, R_mocap(5, 5) = R_mocap_q, R_mocap(6, 6) = R_mocap_q;

  Eigen::MatrixXd R_servo = Eigen::MatrixXd::Zero(4, 4);
  R_servo(0, 0) = R_servo_a, R_servo(1, 1) = R_servo_a, R_servo(2, 2) = R_servo_a, R_servo(3, 3) = R_servo_a;

  Eigen::MatrixXd P_init = Eigen::MatrixXd::Identity(NX, NX);

  Eigen::VectorXd x_init = Eigen::VectorXd::Zero(NX);
  x_init(6) = 1.0;  // q.w

  ekf_.init(ekf_ts, Q, R_imu, R_mocap, R_servo, P_init, x_init);
}

void nmpc::TiltQdServoNMPCwEKF::callbackImu(const spinal::ImuConstPtr& msg)
{
  // TODO:   ekf_.updateIMU(z);

  // time update  TODO: put this part after mpc_solver_ptr_->solve() in the future
  Eigen::VectorXd x_eigen = ekf_.getX();
  std::vector<double> x = EKFEstimator::eigenVec2StdVec(x_eigen);

  std::vector<double> u(sim_solver_ptr_->NU_, 0.0);
  for (int i = 0; i < sim_solver_ptr_->NU_; i++)
    u[i] = getCommand(i);

  sim_solver_ptr_->solve(x, u);

  int& NX = sim_solver_ptr_->NX_;
  Eigen::VectorXd xo = EKFEstimator::stdVec2EigenVec(sim_solver_ptr_->xo_);
  Eigen::MatrixXd A = EKFEstimator::stdVec2EigenMat(sim_solver_ptr_->getMatrixA(), NX, NX);
  ekf_.predict(xo, A);
}

void nmpc::TiltQdServoNMPCwEKF::callbackMoCap(const geometry_msgs::PoseStampedConstPtr& msg)
{
  Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
  z(0) = msg->pose.position.x;
  z(1) = msg->pose.position.y;
  z(2) = msg->pose.position.z;
  z(3) = msg->pose.orientation.w;
  z(4) = msg->pose.orientation.x;
  z(5) = msg->pose.orientation.y;
  z(6) = msg->pose.orientation.z;
  ekf_.updateMoCap(z);
}

void nmpc::TiltQdServoNMPCwEKF::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  TiltQdServoNMPC::callbackJointStates(msg);
  ekf_.updateServo(EKFEstimator::stdVec2EigenVec(joint_angles_));
}

std::vector<double> nmpc::TiltQdServoNMPCwEKF::meas2VecX()
{
  calcDisturbWrench();

  // get bx0
  Eigen::VectorXd x_eigen = ekf_.getX();
  std::vector<double> bx0 = EKFEstimator::eigenVec2StdVec(x_eigen);

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
