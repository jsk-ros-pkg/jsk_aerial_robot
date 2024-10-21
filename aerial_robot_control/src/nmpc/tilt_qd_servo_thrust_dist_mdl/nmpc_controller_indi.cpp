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

  imu_acc_filters_.resize(3);

  for (int i = 0; i < 3; i++)
  {  // butterworth filter 2nd order with 12hz cutoff frequency
    std::vector<std::vector<double>> sos = { { 1.0, 2.0, 1.0, 1.0, -1.4755, 0.5869 } };
    std::vector<double> g = { 0.0279, 1 };
    imu_acc_filters_[i].init(sos, g);
  }

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
  Eigen::VectorXd ft_cmd(motor_num_);
  Eigen::VectorXd ft_meas(motor_num_);
  Eigen::VectorXd ft_mpc(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    ft_cmd(i) = getCommand(i);
    ft_meas(i) = thrust_meas_[i];
  }
  ft_mpc = ft_meas + t_nmpc_samp_ / ts_rotor_ * (ft_cmd - ft_meas);

  Eigen::VectorXd alpha_cmd(joint_num_);
  Eigen::VectorXd alpha_meas(joint_num_);
  Eigen::VectorXd alpha_mpc(joint_num_);
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

  if (alloc_mat_.size() == 0)  // if alloc_mat is not initialized, do not use INDI
    return;

  Eigen::VectorXd wBu_mpc = alloc_mat_ * z_mpc;

  // step 3. calculate the inverse of allocation matrix from fBuMPC (consider the servo dynamics) to u.
  // TODO: maybe I should use the current state of thrust and servo angle to align with the original INDI paper?
  Eigen::VectorXd u_cmd(motor_num_ + joint_num_);
  u_cmd << ft_cmd, alpha_cmd;
  Eigen::VectorXd u_meas(motor_num_ + joint_num_);
  u_meas << ft_meas, alpha_meas;

  Eigen::MatrixXd B_inv;
  if (wBu_mpc.transpose() * wBu_mpc != 0)
    B_inv = u_cmd * ((1 / (wBu_mpc.transpose() * wBu_mpc)) * wBu_mpc.transpose());  // pseudo inverse
  else
    B_inv = u_cmd * (0 * wBu_mpc.transpose());

  // step 4. incremental nonlinear dynamic inversion
  Eigen::Vector3d sf_b_imu(imu_acc_filters_[0].update(msg->acc_data[0]), imu_acc_filters_[1].update(msg->acc_data[1]),
                           imu_acc_filters_[2].update(msg->acc_data[2]));
  Eigen::VectorXd wB_meas(6);
  wB_meas << mass_ * sf_b_imu, Eigen::Vector3d::Zero();
  // we must use wB_meas to get correct allocation result.  TODO: add angular acc

  auto delta_u = B_inv * (wBu_mpc - wB_meas);
  auto u_indi = u_meas + delta_u;

  // step 5. send the command to the robot
  // add a new vector the same size as u_indi
  Eigen::VectorXd u(joint_num_ + motor_num_);

  if (estimator_->getPos(Frame::COG, estimate_mode_).z() < 0.2)  // INDI is not use because of the contact in takeoff
  {
    for (int i = 0; i < joint_num_ + motor_num_; i++)
      u(i) = getCommand(i);
  }
  else
  {
    u = u_indi;
    std::cout << "wBu_mpc: " << wBu_mpc << std::endl;
    std::cout << "wB_meas: " << wB_meas << std::endl;
    std::cout << "delta_u: " << delta_u << std::endl;
  }

  for (int i = 0; i < motor_num_; i++)
    flight_cmd_.base_thrust[i] = (float)u(i);

  gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(u(motor_num_ + i));
  }

  pub_flight_cmd_.publish(flight_cmd_);
  pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoThrustNMPCwINDI, aerial_robot_control::ControlBase);
