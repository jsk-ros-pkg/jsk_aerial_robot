//
// Created by jinjie on 24/08/01.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_mdl/nmpc_controller_indi.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoThrustNMPCwINDI::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                  double ctrl_loop_du)
{
  TiltQdServoThrustDistNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  sub_imu_ = nh_.subscribe("imu", 1, &TiltQdServoThrustNMPCwINDI::callbackImu, this);
}

void nmpc::TiltQdServoThrustNMPCwINDI::initParams()
{
  TiltQdServoThrustDistNMPC::initParams();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  getParam<double>(nmpc_nh, "t_servo", ts_servo_, 0.1);
  getParam<double>(nmpc_nh, "t_rotor", ts_rotor_, 0.1);
}

void nmpc::TiltQdServoThrustNMPCwINDI::calcDisturbWrench()
{
}

void nmpc::TiltQdServoThrustNMPCwINDI::callbackImu(const spinal::ImuConstPtr& msg)
{
  /* INDI */
  // step 1. calculate the rotor thrust and servo angle after t_nmpc_samp_, which considers the actuators' dynamics.
  Eigen::VectorXd ft_cmd = Eigen::VectorXd::Zero(motor_num_);
  Eigen::VectorXd ft_meas = Eigen::VectorXd::Zero(motor_num_);
  Eigen::VectorXd ft_mpc = Eigen::VectorXd::Zero(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    ft_cmd(i) = getCommand(i);
    ft_meas(i) = thrust_meas_[i];
  }
  ft_mpc = ft_meas + t_nmpc_samp_ / ts_rotor_ * (ft_cmd - ft_meas);

  Eigen::VectorXd alpha_cmd = Eigen::VectorXd::Zero(joint_num_);
  Eigen::VectorXd alpha_meas = Eigen::VectorXd::Zero(joint_num_);
  Eigen::VectorXd alpha_mpc = Eigen::VectorXd::Zero(joint_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    alpha_cmd(i) = getCommand(i + motor_num_);
    alpha_meas(i) = joint_angles_[i];
  }
  alpha_mpc = alpha_meas + t_nmpc_samp_ / ts_servo_ * (alpha_cmd - alpha_meas);

  // step 2. calculate fBu and tauBu from the delayed rotor thrust (ft_mpc) and servo angle (alpha_mpc)
  if (motor_num_ != joint_num_)
    throw std::runtime_error(
        "The number of motors and joints should be the same. If not same, try to use other methods.");

  Eigen::VectorXd z_mpc = Eigen::VectorXd::Zero(motor_num_ + joint_num_);
  for (int i = 0; i < z_mpc.size() / 2; i++)
  {
    z_mpc(2 * i) = ft_mpc(i) * sin(alpha_mpc(i));
    z_mpc(2 * i + 1) = ft_mpc(i) * cos(alpha_mpc(i));
  }

  auto wBu_mpc = alloc_mat_ * z_mpc;

  auto fBu_mpc = wBu_mpc.head(3);
  auto tauBu_mpc = wBu_mpc.tail(3);

  // step 3. calculate the inverse of allocation matrix from fBuMPC (consider the servo dynamics) to u.
  // TODO: maybe I should use the current state of thrust and servo angle to align with the original INDI paper?
  Eigen::VectorXd u_cmd(motor_num_ + joint_num_);
  u_cmd << ft_cmd, alpha_cmd;
  Eigen::VectorXd u_meas(motor_num_ + joint_num_);
  u_meas << ft_meas, alpha_meas;

  Eigen::MatrixXd B_inv = u_cmd * ((1 / (fBu_mpc.transpose() * fBu_mpc)) * fBu_mpc.transpose());  // pseudo inverse

  // step 4. incremental nonlinear dynamic inversion
  Eigen::Vector3d sf_b_imu(msg->acc_data[0], msg->acc_data[1], msg->acc_data[2]);

  auto delta_u = B_inv * (fBu_mpc - mass_ * sf_b_imu);
  auto u_indi = u_meas + delta_u;

  // step 5. send the command to the robot
  for (int i = 0; i < motor_num_; i++)
    flight_cmd_.base_thrust[i] = (float)u_indi(i);

  gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(u_indi(motor_num_ + i));
  }

  pub_flight_cmd_.publish(flight_cmd_);
  pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoThrustNMPCwINDI, aerial_robot_control::ControlBase);
