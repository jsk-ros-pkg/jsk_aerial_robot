// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/27.
//

#include "aerial_robot_control/nmpc/fix_qd_angvel_out_mdl/nmpc_controller.h"

using namespace aerial_robot_control;

nmpc_under_act_body_rate::NMPCController::NMPCController() : target_roll_(0), target_pitch_(0), candidate_yaw_term_(0)
{
}

void nmpc_under_act_body_rate::NMPCController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  ros::NodeHandle control_nh(nh_, "controller");

  mass_ = robot_model_->getMass();
  // TODO: wired. I think the original value should be -9.8 in ENU frame
  gravity_const_ = robot_model_->getGravity()[2];

  // get params and initialize nmpc solver
  PhysicalParams physical_params{};
  physical_params.mass = robot_model_->getMass();
  physical_params.gravity = robot_model_->getGravity()[2];
  physical_params.c_thrust_max = robot_model_->getThrustUpperLimit() * robot_model_->getRotorNum();
  physical_params.c_thrust_min = robot_model_->getThrustLowerLimit() * robot_model_->getRotorNum();
  getParam<double>(control_nh, "nmpc/v_max", physical_params.v_max, 0.5);
  getParam<double>(control_nh, "nmpc/v_min", physical_params.v_min, -0.5);
  getParam<double>(control_nh, "nmpc/w_max", physical_params.w_max, 3.0);
  getParam<double>(control_nh, "nmpc/w_min", physical_params.w_min, -3.0);
  getParam<double>(control_nh, "nmpc/T_samp", t_nmpc_samp_, 0.025);
  getParam<double>(control_nh, "nmpc/T_step", t_nmpc_integ_, 0.1);
  mpc_solver_.initialize(physical_params);

  getParam<double>(control_nh, "nmpc/yaw_p_gain", yaw_p_gain, 40.0);
  getParam<double>(control_nh, "nmpc/yaw_d_gain", yaw_d_gain, 1.0);

  getParam<bool>(control_nh, "nmpc/is_attitude_ctrl", is_attitude_ctrl_, true);
  getParam<bool>(control_nh, "nmpc/is_body_rate_ctrl", is_body_rate_ctrl_, false);

  /* timers */
  tmr_viz_ = nh_.createTimer(ros::Duration(0.05), &NMPCController::callbackViz, this);

  /* publishers */
  pub_viz_pred_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_pred", 1);
  pub_viz_ref_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_ref", 1);
  pub_flight_cmd_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);

  pub_rpy_gain_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);  // tmp
  pub_p_matrix_pseudo_inverse_inertia_ =
      nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);  // tmp

  /* services */
  srv_set_control_mode_ = nh_.serviceClient<spinal::SetControlMode>("set_control_mode");
  bool res = ros::service::waitForService("set_control_mode", ros::Duration(5));

  /* init some values */
  odom_ = nav_msgs::Odometry();
  odom_.pose.pose.orientation.w = 1;
  reset();

  /* set control mode */
  if (!res)
  {
    ROS_ERROR("cannot find service named set_control_mode");
  }
  ros::Duration(2.0).sleep();
  spinal::SetControlMode set_control_mode_srv;
  set_control_mode_srv.request.is_attitude = is_attitude_ctrl_;
  set_control_mode_srv.request.is_body_rate = is_body_rate_ctrl_;
  while (!srv_set_control_mode_.call(set_control_mode_srv))
    ROS_WARN_THROTTLE(1,
                      "Waiting for set_control_mode service.... If you always see this message, the robot cannot fly.");

  ROS_INFO("Set control mode: attitude = %d and body rate = %d", set_control_mode_srv.request.is_attitude,
           set_control_mode_srv.request.is_body_rate);

  ROS_INFO("MPC Controller initialized!");
}

bool nmpc_under_act_body_rate::NMPCController::update()
{
  if (!ControlBase::update())
    return false;

  this->controlCore();
  this->sendCmd();

  return true;
}

void nmpc_under_act_body_rate::NMPCController::reset()
{
  ControlBase::reset();

  sendRPYGain();                // tmp for angular gains  TODO: change to angular gains only
  sendRotationalInertiaComp();  // tmp for inertia

  /* reset controller using odom */
  tf::Vector3 pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 rpy_ = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Quaternion q;
  q.setRPY(rpy_.x(), rpy_.y(), rpy_.z());

  double x[NX] = { pos_.x(), pos_.y(), pos_.z(), vel_.x(), vel_.y(), vel_.z(), q.w(), q.x(), q.y(), q.z() };
  double u[NU] = { 0.0, 0.0, 0.0, gravity_const_ };
  initPredXU(x_u_ref_);
  for (int i = 0; i < NN; i++)
  {
    std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);

  mpc_solver_.reset(x_u_ref_);

  /* reset flight_cmd, only focus on the thrust cmd to prevent sudden change when taking off */
  flight_cmd_.base_thrust = std::vector<float>(motor_num_, 0.0);
}

nav_msgs::Odometry nmpc_under_act_body_rate::NMPCController::getOdom()
{
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);
  tf::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = pos.x();
  odom.pose.pose.position.y = pos.y();
  odom.pose.pose.position.z = pos.z();
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();
  odom.twist.twist.angular.x = omega.x();
  odom.twist.twist.angular.y = omega.y();
  odom.twist.twist.angular.z = omega.z();

  return odom;
}

void nmpc_under_act_body_rate::NMPCController::controlCore()
{
  /* get odom information */
  nav_msgs::Odometry odom_now = getOdom();

  // check the sign of the quaternion, avoid the flip of the quaternion TODO: should be moved to Estimator
  double qe_c_w = odom_now.pose.pose.orientation.w * odom_.pose.pose.orientation.w +
                  odom_now.pose.pose.orientation.x * odom_.pose.pose.orientation.x +
                  odom_now.pose.pose.orientation.y * odom_.pose.pose.orientation.y +
                  odom_now.pose.pose.orientation.z * odom_.pose.pose.orientation.z;
  if (qe_c_w < 0)
  {
    odom_now.pose.pose.orientation.w = -odom_now.pose.pose.orientation.w;
    odom_now.pose.pose.orientation.x = -odom_now.pose.pose.orientation.x;
    odom_now.pose.pose.orientation.y = -odom_now.pose.pose.orientation.y;
    odom_now.pose.pose.orientation.z = -odom_now.pose.pose.orientation.z;
  }

  odom_ = odom_now;

  /* set target */
  tf::Vector3 target_pos_ = navigator_->getTargetPos();
  double x[NX] = { target_pos_.x(), target_pos_.y(), target_pos_.z(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
  double u[NU] = { 0.0, 0.0, 0.0, gravity_const_ };

  // Aim: gently add the target point to the end of the reference trajectory
  // - x: NN + 1, u: NN
  // - for 0 ~ NN-2 x and u, shift
  // - copy x to x: NN-1 and NN, copy u to u: NN-1
  for (int i = 0; i < NN - 1; i++)
  {
    // shift one step
    std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
              x_u_ref_.x.data.begin() + NX * i);
    std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
              x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * (NN - 1));
  std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * (NN - 1));

  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);

  /* solve */
  mpc_solver_.solve(odom_, x_u_ref_);

  /* get result */
  // convert quaternion to rpy
  tf::Quaternion q0, q1;
  q0.setW(mpc_solver_.x_u_out_.x.data.at(6));
  q0.setX(mpc_solver_.x_u_out_.x.data.at(7));
  q0.setY(mpc_solver_.x_u_out_.x.data.at(8));
  q0.setZ(mpc_solver_.x_u_out_.x.data.at(9));
  q1.setW(mpc_solver_.x_u_out_.x.data.at(6 + NX));
  q1.setX(mpc_solver_.x_u_out_.x.data.at(7 + NX));
  q1.setY(mpc_solver_.x_u_out_.x.data.at(8 + NX));
  q1.setZ(mpc_solver_.x_u_out_.x.data.at(9 + NX));

  double rpy0[3], rpy1[3];
  tf::Matrix3x3(q0).getRPY(rpy0[0], rpy0[1], rpy0[2]);
  tf::Matrix3x3(q1).getRPY(rpy1[0], rpy1[1], rpy1[2]);

  // - roll, pitch angles
  target_roll_ = (t_nmpc_samp_ * rpy0[0] + (t_nmpc_integ_ - t_nmpc_samp_) * rpy1[0]) / t_nmpc_integ_;
  target_pitch_ = (t_nmpc_samp_ * rpy0[1] + (t_nmpc_integ_ - t_nmpc_samp_) * rpy1[1]) / t_nmpc_integ_;

  flight_cmd_.angles[0] = static_cast<float>(target_roll_);
  flight_cmd_.angles[1] = static_cast<float>(target_pitch_);

  // - yaw angle
  double target_yaw, error_yaw, yaw_p_term, error_yaw_rate, yaw_d_term;
  target_yaw = (t_nmpc_samp_ * rpy0[2] + (t_nmpc_integ_ - t_nmpc_samp_) * rpy1[2]) / t_nmpc_integ_;
  error_yaw = angles::shortest_angular_distance(target_yaw, rpy0[2]);
  yaw_p_term = yaw_p_gain * error_yaw;
  candidate_yaw_term_ = -yaw_p_term;

  flight_cmd_.angles[2] = static_cast<float>(candidate_yaw_term_);

  // - body rates
  flight_cmd_.body_rates[0] = static_cast<float>(mpc_solver_.x_u_out_.u.data.at(0));
  flight_cmd_.body_rates[1] = static_cast<float>(mpc_solver_.x_u_out_.u.data.at(1));
  flight_cmd_.body_rates[2] = static_cast<float>(mpc_solver_.x_u_out_.u.data.at(2));

  // - thrust
  double acc_body_z = mpc_solver_.x_u_out_.u.data.at(3);

  Eigen::VectorXd static_thrust = robot_model_->getStaticThrust();
  Eigen::VectorXd g = robot_model_->getGravity();
  Eigen::VectorXd target_thrusts = acc_body_z * static_thrust / g.norm();

  for (int i = 0; i < motor_num_; i++)
  {
    flight_cmd_.base_thrust[i] = static_cast<float>(target_thrusts(i));
  }
}

void nmpc_under_act_body_rate::NMPCController::sendCmd()
{
  pub_flight_cmd_.publish(flight_cmd_);
}

/**
 * @brief callbackViz: publish the predicted trajectory and reference trajectory
 * @param [ros::TimerEvent&] event
 */
void nmpc_under_act_body_rate::NMPCController::callbackViz(const ros::TimerEvent& event)
{
  // from mpc_solver_.x_u_out to PoseArray
  geometry_msgs::PoseArray pred_poses;
  geometry_msgs::PoseArray ref_poses;

  for (int i = 0; i < NN; ++i)
  {
    geometry_msgs::Pose pred_pose;
    pred_pose.position.x = mpc_solver_.x_u_out_.x.data.at(i * NX);
    pred_pose.position.y = mpc_solver_.x_u_out_.x.data.at(i * NX + 1);
    pred_pose.position.z = mpc_solver_.x_u_out_.x.data.at(i * NX + 2);
    pred_pose.orientation.w = mpc_solver_.x_u_out_.x.data.at(i * NX + 6);
    pred_pose.orientation.x = mpc_solver_.x_u_out_.x.data.at(i * NX + 7);
    pred_pose.orientation.y = mpc_solver_.x_u_out_.x.data.at(i * NX + 8);
    pred_pose.orientation.z = mpc_solver_.x_u_out_.x.data.at(i * NX + 9);
    pred_poses.poses.push_back(pred_pose);

    geometry_msgs::Pose ref_pose;
    ref_pose.position.x = x_u_ref_.x.data.at(i * NX);
    ref_pose.position.y = x_u_ref_.x.data.at(i * NX + 1);
    ref_pose.position.z = x_u_ref_.x.data.at(i * NX + 2);
    ref_pose.orientation.w = x_u_ref_.x.data.at(i * NX + 6);
    ref_pose.orientation.x = x_u_ref_.x.data.at(i * NX + 7);
    ref_pose.orientation.y = x_u_ref_.x.data.at(i * NX + 8);
    ref_pose.orientation.z = x_u_ref_.x.data.at(i * NX + 9);
    ref_poses.poses.push_back(ref_pose);
  }

  pred_poses.header.frame_id = "world";
  pred_poses.header.stamp = ros::Time::now();
  pub_viz_pred_.publish(pred_poses);

  ref_poses.header.frame_id = "world";
  ref_poses.header.stamp = ros::Time::now();
  pub_viz_ref_.publish(ref_poses);
}
void nmpc_under_act_body_rate::NMPCController::sendRPYGain()
{
  spinal::RollPitchYawTerms rpy_gain_msg;
  rpy_gain_msg.motors.resize(motor_num_);

  rpy_gain_msg.motors[0].roll_p = -2303;
  rpy_gain_msg.motors[0].roll_i = -354;
  rpy_gain_msg.motors[0].roll_d = -395;
  rpy_gain_msg.motors[0].pitch_p = 2297;
  rpy_gain_msg.motors[0].pitch_i = 353;
  rpy_gain_msg.motors[0].pitch_d = 393;
  rpy_gain_msg.motors[0].yaw_d = 1423;

  rpy_gain_msg.motors[1].roll_p = -2291;
  rpy_gain_msg.motors[1].roll_i = -352;
  rpy_gain_msg.motors[1].roll_d = -390;
  rpy_gain_msg.motors[1].pitch_p = -2297;
  rpy_gain_msg.motors[1].pitch_i = -353;
  rpy_gain_msg.motors[1].pitch_d = -393;
  rpy_gain_msg.motors[1].yaw_d = -1426;

  rpy_gain_msg.motors[2].roll_p = 2291;
  rpy_gain_msg.motors[2].roll_i = 352;
  rpy_gain_msg.motors[2].roll_d = 390;
  rpy_gain_msg.motors[2].pitch_p = -2297;
  rpy_gain_msg.motors[2].pitch_i = -353;
  rpy_gain_msg.motors[2].pitch_d = -393;
  rpy_gain_msg.motors[2].yaw_d = 1426;

  rpy_gain_msg.motors[3].roll_p = 2303;
  rpy_gain_msg.motors[3].roll_i = 354;
  rpy_gain_msg.motors[3].roll_d = 395;
  rpy_gain_msg.motors[3].pitch_p = 2297;
  rpy_gain_msg.motors[3].pitch_i = 353;
  rpy_gain_msg.motors[3].pitch_d = 393;
  rpy_gain_msg.motors[3].yaw_d = -1423;

  if (!is_attitude_ctrl_ && is_body_rate_ctrl_)
  {
    ros::NodeHandle control_nh(nh_, "controller");
    double roll_rate_p_gain, pitch_rate_p_gain, yaw_rate_p_gain;
    getParam<double>(control_nh, "nmpc/roll_rate_p_gain", roll_rate_p_gain, 0.4);
    getParam<double>(control_nh, "nmpc/pitch_rate_p_gain", pitch_rate_p_gain, 0.4);
    getParam<double>(control_nh, "nmpc/yaw_rate_p_gain", yaw_rate_p_gain, 1.5);

    rpy_gain_msg.motors[0].roll_d = (short)(-roll_rate_p_gain * 1000);
    rpy_gain_msg.motors[0].pitch_d = (short)(pitch_rate_p_gain * 1000);
    rpy_gain_msg.motors[0].yaw_d = (short)(yaw_rate_p_gain * 1000);

    rpy_gain_msg.motors[1].roll_d = (short)(-roll_rate_p_gain * 1000);
    rpy_gain_msg.motors[1].pitch_d = (short)(-pitch_rate_p_gain * 1000);
    rpy_gain_msg.motors[1].yaw_d = (short)(-yaw_rate_p_gain * 1000);

    rpy_gain_msg.motors[2].roll_d = (short)(roll_rate_p_gain * 1000);
    rpy_gain_msg.motors[2].pitch_d = (short)(-pitch_rate_p_gain * 1000);
    rpy_gain_msg.motors[2].yaw_d = (short)(yaw_rate_p_gain * 1000);

    rpy_gain_msg.motors[3].roll_d = (short)(roll_rate_p_gain * 1000);
    rpy_gain_msg.motors[3].pitch_d = (short)(pitch_rate_p_gain * 1000);
    rpy_gain_msg.motors[3].yaw_d = (short)(-yaw_rate_p_gain * 1000);
  }

  pub_rpy_gain_.publish(rpy_gain_msg);
}

void nmpc_under_act_body_rate::NMPCController::sendRotationalInertiaComp()
{
  int lqi_mode_ = 4;

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));

  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;  // to spinal
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for (int i = 0; i < motor_num_; ++i)
  {
    /* the p matrix pseudo inverse and inertia */
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
    if (lqi_mode_ == 4)
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;
    else
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0;
  }

  /* the articulated inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  pub_p_matrix_pseudo_inverse_inertia_.publish(p_pseudo_inverse_with_inertia_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc_under_act_body_rate::NMPCController,
                       aerial_robot_control::ControlBase);
