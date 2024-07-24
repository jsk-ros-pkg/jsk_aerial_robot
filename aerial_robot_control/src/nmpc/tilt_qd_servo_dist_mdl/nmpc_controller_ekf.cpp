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
  sub_mocap_ = nh_.subscribe("mocap/pose", 1, &TiltQdServoNMPCwEKF::callbackMoCap, this);
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
  /* time update */  // TODO: put this part after mpc_solver_ptr_->solve() in the future
  std::vector<double> x(sim_solver_ptr_->NX_, 0.0);
  Eigen::VectorXd x_eigen = ekf_.getX();
  x = EKFEstimator::eigenVec2StdVec(x_eigen);

  std::vector<double> u(sim_solver_ptr_->NU_, 0.0);
  for (int i = 0; i < sim_solver_ptr_->NU_; i++)
    u[i] = getCommand(i);

  // for ground contact
  // if the robot is on the ground, set the disturbance to only support force
  uint8_t navi_state = navigator_->getNaviState();
  if ((navi_state != aerial_robot_navigation::TAKEOFF_STATE) && (navi_state != aerial_robot_navigation::LAND_STATE) &&
      (navi_state != aerial_robot_navigation::HOVER_STATE))
  {
    is_on_ground_ = true;
    x.at(5) = 0.0;  // vz
    x.at(17) = 0.0;
    x.at(18) = 0.0;
    x.at(19) = mass_ * gravity_const_;
    x.at(20) = 0.0;
    x.at(21) = 0.0;
    x.at(22) = 0.0;
    std::fill(u.begin(), u.end(), 0.0);
  }
  else
  {
    if (is_on_ground_)  // the switch from on ground to takeoff
    {
      is_on_ground_ = false;
      x.at(17) = 0.0;
      x.at(18) = 0.0;
      x.at(19) = 0.0;
      x.at(20) = 0.0;
      x.at(21) = 0.0;
      x.at(22) = 0.0;
    }
  }

//  std::cout << "x = ";
//  for (int i = 0; i < x.size(); i++)
//    std::cout << x[i] << " ";
//  std::cout << std::endl;

  sim_solver_ptr_->solve(x, u);

  int& NX = sim_solver_ptr_->NX_;
  Eigen::VectorXd xo = EKFEstimator::stdVec2EigenVec(sim_solver_ptr_->xo_);
  Eigen::MatrixXd A = EKFEstimator::stdVec2EigenMat(sim_solver_ptr_->getMatrixA(), NX, NX);
  ekf_.predict(xo, A);

  /* IMU update */
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Vector3d p1_b = rotor_p[0];
  Eigen::Vector3d p2_b = rotor_p[1];
  Eigen::Vector3d p3_b = rotor_p[2];
  Eigen::Vector3d p4_b = rotor_p[3];

  //  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  //  int dr1 = rotor_dr.find(1)->second;
  //  int dr2 = rotor_dr.find(2)->second;
  //  int dr3 = rotor_dr.find(3)->second;
  //  int dr4 = rotor_dr.find(4)->second;

  double kq_d_kt = robot_model_->getThrustWrenchUnits()[0][5];

  double sqrt_p1b_xy = sqrt(p1_b.x() * p1_b.x() + p1_b.y() * p1_b.y());
  double sqrt_p2b_xy = sqrt(p2_b.x() * p2_b.x() + p2_b.y() * p2_b.y());
  double sqrt_p3b_xy = sqrt(p3_b.x() * p3_b.x() + p3_b.y() * p3_b.y());
  double sqrt_p4b_xy = sqrt(p4_b.x() * p4_b.x() + p4_b.y() * p4_b.y());

  double ft1 = getCommand(0);
  double ft2 = getCommand(1);
  double ft3 = getCommand(2);
  double ft4 = getCommand(3);
  double alpha1 = getCommand(4);
  double alpha2 = getCommand(5);
  double alpha3 = getCommand(6);
  double alpha4 = getCommand(7);

  // sf = (fBu+RBI.fId)/m
  Eigen::Vector3d fBu = Eigen::Vector3d::Zero();
  fBu(0) = ft1 * p1_b.y() * sin(alpha1) / sqrt_p1b_xy + ft2 * p2_b.y() * sin(alpha2) / sqrt_p2b_xy +
           ft3 * p3_b.y() * sin(alpha3) / sqrt_p3b_xy + ft4 * p4_b.y() * sin(alpha4) / sqrt_p4b_xy;
  fBu(1) = -ft1 * p1_b.x() * sin(alpha1) / sqrt_p1b_xy - ft2 * p2_b.x() * sin(alpha2) / sqrt_p2b_xy -
           ft3 * p3_b.x() * sin(alpha3) / sqrt_p3b_xy - ft4 * p4_b.x() * sin(alpha4) / sqrt_p4b_xy;
  fBu(2) = ft1 * cos(alpha1) + ft2 * cos(alpha2) + ft3 * cos(alpha3) + ft4 * cos(alpha4);

  Eigen::Vector3d fId = Eigen::Vector3d::Zero();
  fId(0) = ekf_.getX(17);
  fId(1) = ekf_.getX(18);
  fId(2) = ekf_.getX(19);

  double qw = ekf_.getX(6);
  double qx = ekf_.getX(7);
  double qy = ekf_.getX(8);
  double qz = ekf_.getX(9);

  Eigen::Matrix3d RIB = Eigen::Matrix3d::Zero();
  RIB(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
  RIB(0, 1) = 2 * qx * qy - 2 * qz * qw;
  RIB(0, 2) = 2 * qx * qz + 2 * qy * qw;
  RIB(1, 0) = 2 * qx * qy + 2 * qz * qw;
  RIB(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
  RIB(1, 2) = 2 * qy * qz - 2 * qx * qw;
  RIB(2, 0) = 2 * qx * qz - 2 * qy * qw;
  RIB(2, 1) = 2 * qy * qz + 2 * qx * qw;
  RIB(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;

  Eigen::Matrix3d RBI = RIB.transpose();

  Eigen::Vector3d sf_est = (fBu + RBI * fId) / mass_;

  // get C_imu_
  Eigen::MatrixXd dsfBIMUdqIB = Eigen::MatrixXd::Zero(3, 4);
  dsfBIMUdqIB(0, 0) = 2 * qz * fId(1) - 2 * qy * fId(2);
  dsfBIMUdqIB(1, 0) = -2 * qz * fId(0) + 2 * qx * fId(2);
  dsfBIMUdqIB(2, 0) = 2 * qy * fId(0) - 2 * qx * fId(1);
  dsfBIMUdqIB(0, 1) = 2 * qy * fId(1) + 2 * qz * fId(2);
  dsfBIMUdqIB(1, 1) = 2 * qy * fId(0) - 4 * qx * fId(1) + 2 * qw * fId(2);
  dsfBIMUdqIB(2, 1) = 2 * qz * fId(0) - 2 * qw * fId(1) - 4 * qx * fId(2);
  dsfBIMUdqIB(0, 2) = -4 * qy * fId(0) + 2 * qx * fId(1) - 2 * qw * fId(2);
  dsfBIMUdqIB(1, 2) = 2 * qx * fId(0) + 2 * qz * fId(2);
  dsfBIMUdqIB(2, 2) = 2 * qw * fId(0) + 2 * qz * fId(1) - 4 * qy * fId(2);
  dsfBIMUdqIB(0, 3) = -4 * qz * fId(0) + 2 * qw * fId(1) + 2 * qx * fId(2);
  dsfBIMUdqIB(1, 3) = -2 * qw * fId(0) - 4 * qz * fId(1) + 2 * qy * fId(2);
  dsfBIMUdqIB(2, 3) = 2 * qx * fId(0) + 2 * qy * fId(1);
  dsfBIMUdqIB /= mass_;

  Eigen::MatrixXd dsfBIMUdA = Eigen::MatrixXd::Zero(3, 4);
  dsfBIMUdA(0, 0) = ft1 * p1_b.y() * cos(alpha1) / mass_ / sqrt_p1b_xy;
  dsfBIMUdA(1, 0) = -ft1 * p1_b.x() * cos(alpha1) / mass_ / sqrt_p1b_xy;
  dsfBIMUdA(2, 0) = -ft1 * sin(alpha1) / mass_;
  dsfBIMUdA(0, 1) = ft2 * p2_b.y() * cos(alpha2) / mass_ / sqrt_p2b_xy;
  dsfBIMUdA(1, 1) = -ft2 * p2_b.x() * cos(alpha2) / mass_ / sqrt_p2b_xy;
  dsfBIMUdA(2, 1) = -ft2 * sin(alpha2) / mass_;
  dsfBIMUdA(0, 2) = ft3 * p3_b.y() * cos(alpha3) / mass_ / sqrt_p3b_xy;
  dsfBIMUdA(1, 2) = -ft3 * p3_b.x() * cos(alpha3) / mass_ / sqrt_p3b_xy;
  dsfBIMUdA(2, 2) = -ft3 * sin(alpha3) / mass_;
  dsfBIMUdA(0, 3) = ft4 * p4_b.y() * cos(alpha4) / mass_ / sqrt_p4b_xy;
  dsfBIMUdA(1, 3) = -ft4 * p4_b.x() * cos(alpha4) / mass_ / sqrt_p4b_xy;
  dsfBIMUdA(2, 3) = -ft4 * sin(alpha4) / mass_;

  Eigen::MatrixXd C_imu = Eigen::MatrixXd::Zero(6, 23);
  for (int i = 0; i < dsfBIMUdqIB.rows(); i++)
  {
    for (int j = 0; j < dsfBIMUdqIB.cols(); j++)
      C_imu(i, 6 + j) = dsfBIMUdqIB(i, j);
  }
  for (int i = 0; i < dsfBIMUdA.rows(); i++)
  {
    for (int j = 0; j < dsfBIMUdA.cols(); j++)
      C_imu(i, 13 + j) = dsfBIMUdA(i, j);
  }
  for (int i = 0; i < RBI.rows(); i++)
  {
    for (int j = 0; j < RBI.cols(); j++)
      C_imu(i, 17 + j) = RBI(i, j) / mass_;
  }
  C_imu(3, 10) = 1;
  C_imu(4, 11) = 1;
  C_imu(5, 12) = 1;

  Eigen::VectorXd z = Eigen::VectorXd::Zero(6);
  z(0) = msg->acc_data.at(0);
  z(1) = msg->acc_data.at(1);
  z(2) = msg->acc_data.at(2);
  z(3) = msg->gyro_data.at(0);
  z(4) = msg->gyro_data.at(1);
  z(5) = msg->gyro_data.at(2);

  Eigen::VectorXd z_est = Eigen::VectorXd::Zero(6);
  z_est(0) = sf_est(0);
  z_est(1) = sf_est(1);
  z_est(2) = sf_est(2);
  z_est(3) = ekf_.getX(10);
  z_est(4) = ekf_.getX(11);
  z_est(5) = ekf_.getX(12);

//  ekf_.updateIMU(z, C_imu, z_est);  // TODO: check the result
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
