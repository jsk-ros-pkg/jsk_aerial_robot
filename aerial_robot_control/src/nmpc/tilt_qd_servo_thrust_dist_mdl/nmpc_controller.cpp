//
// Created by jinjie on 24/07/31.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_mdl/nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoThrustDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                 double ctrl_loop_du)
{
  TiltQdServoDistNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &TiltQdServoThrustDistNMPC::callbackESCTelem, this);
}

void nmpc::TiltQdServoThrustDistNMPC::initParams()
{
  TiltQdServoDistNMPC::initParams();

  ros::NodeHandle motor_nh(nh_, "motor_info");
  getParam<double>(motor_nh, "krpm_rate", krpm2_d_thrust_, 0.0);
}

void nmpc::TiltQdServoThrustDistNMPC::initCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qt, Rtc_d, Rac_d;
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 300);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 400);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 300);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 300);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 5);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 5);
  getParam<double>(nmpc_nh, "Qa", Qa, 1);
  getParam<double>(nmpc_nh, "Qt", Qt, 1);

  getParam<double>(nmpc_nh, "Rtc_d", Rtc_d, 1);
  getParam<double>(nmpc_nh, "Rac_d", Rac_d, 250);

  // diagonal matrix
  mpc_solver_ptr_->setCostWDiagElement(0, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(1, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(2, Qp_z);
  mpc_solver_ptr_->setCostWDiagElement(3, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(4, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(5, Qv_z);
  mpc_solver_ptr_->setCostWDiagElement(6, 0);
  mpc_solver_ptr_->setCostWDiagElement(7, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(8, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(9, Qq_z);
  mpc_solver_ptr_->setCostWDiagElement(10, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(11, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(12, Qw_z);
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtc_d, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);

  mpc_solver_ptr_->setCostWeight(true, true);
}

void nmpc::TiltQdServoThrustDistNMPC::callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
{
  double krpm = (double)msg->esc_telemetry_1.rpm * 0.001;
  thrust_meas_[0] = krpm * krpm / krpm2_d_thrust_;

  krpm = (double)msg->esc_telemetry_2.rpm * 0.001;
  thrust_meas_[1] = krpm * krpm / krpm2_d_thrust_;

  krpm = (double)msg->esc_telemetry_3.rpm * 0.001;
  thrust_meas_[2] = krpm * krpm / krpm2_d_thrust_;

  krpm = (double)msg->esc_telemetry_4.rpm * 0.001;
  thrust_meas_[3] = krpm * krpm / krpm2_d_thrust_;
}

void nmpc::TiltQdServoThrustDistNMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                                   const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                                   const Eigen::VectorXd& ref_wrench_b, vector<double>& x,
                                                   vector<double>& u) const
{
  x.at(0) = ref_pos_i.x();
  x.at(1) = ref_pos_i.y();
  x.at(2) = ref_pos_i.z();
  x.at(3) = ref_vel_i.x();
  x.at(4) = ref_vel_i.y();
  x.at(5) = ref_vel_i.z();
  x.at(6) = ref_quat_ib.w();
  x.at(7) = ref_quat_ib.x();
  x.at(8) = ref_quat_ib.y();
  x.at(9) = ref_quat_ib.z();
  x.at(10) = ref_omega_b.x();
  x.at(11) = ref_omega_b.y();
  x.at(12) = ref_omega_b.z();
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  for (int i = 0; i < x_lambda.size() / 2; i++)
  {
    double a_ref = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));
    x.at(13 + i) = a_ref;
    double ft_ref = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    x.at(13 + x_lambda.size() / 2 + i) = ft_ref;
  }
}

std::vector<double> nmpc::TiltQdServoThrustDistNMPC::meas2VecX()
{
  auto bx0 = TiltMtServoNMPC::meas2VecX();

  for (int i = 0; i < motor_num_; i++)
    bx0[13 + joint_num_ + i] = thrust_meas_[i];

  /* disturbance rejection */
  calcDisturbWrench();
  bx0[13 + joint_num_ + motor_num_ + 0] = dist_force_w_.x;
  bx0[13 + joint_num_ + motor_num_ + 1] = dist_force_w_.y;
  bx0[13 + joint_num_ + motor_num_ + 2] = dist_force_w_.z;
  bx0[13 + joint_num_ + motor_num_ + 3] = dist_torque_cog_.x;
  bx0[13 + joint_num_ + motor_num_ + 4] = dist_torque_cog_.y;
  bx0[13 + joint_num_ + motor_num_ + 5] = dist_torque_cog_.z;
  return bx0;
}

void nmpc::TiltQdServoThrustDistNMPC::cfgNMPCCallback(aerial_robot_control::NMPCConfig& config, uint32_t level)
{
  // TODO: finish this part.
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoThrustDistNMPC, aerial_robot_control::ControlBase);
