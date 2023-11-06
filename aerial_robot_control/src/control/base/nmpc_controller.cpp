// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/27.
//

#include "aerial_robot_control/control/base/nmpc_controller.h"

aerial_robot_control::NMPCController::NMPCController() : target_roll_(0), target_pitch_(0), candidate_yaw_term_(0)
{
}

void aerial_robot_control::NMPCController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  mpc_solver_.initialize();

  ros::NodeHandle control_nh(nh_, "controller");

  mass_ = robot_model_->getMass();
  // TODO: wired. I think the original value should be -9.8 in ENU frame
  gravity_const_ = robot_model_->getGravity()[2];

  /* timers */
  tmr_viz_ = nh_.createTimer(ros::Duration(0.05), &NMPCController::callback_viz, this);

  /* subscribers */

  /* publishers */
  pub_viz_pred_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_pred", 1);
  pub_viz_ref_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_ref", 1);
  pub_flight_cmd_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);

  // get these parameters from rosparam
  //  ros::param::get("~has_traj_server", has_traj_server_);
  //  ros::param::get("~has_pred_viz", has_pred_viz_);
  //  ros::param::get("~pred_viz_type", pred_viz_type_);
  //  ros::param::get("~is_build_acados", is_build_acados_);
  //  ros::param::get("~has_pred_pub", has_pred_pub_);

  reset();

  ROS_INFO("MPC Controller initialized!");
}

bool aerial_robot_control::NMPCController::update()
{
  if (!ControlBase::update())
    return false;

  this->controlCore();
  this->sendCmd();

  // output flight cmd using ROS INFO
  ROS_INFO("r: %f, p: %f, y: %f, 1/4 thrust: %f", flight_cmd_.angles[0], flight_cmd_.angles[1], flight_cmd_.angles[2],
           flight_cmd_.base_thrust[0]);

  return true;
}

void aerial_robot_control::NMPCController::reset()
{
  ControlBase::reset();

  tf::Vector3 pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 rpy_ = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Quaternion q;
  q.setRPY(rpy_.x(), rpy_.y(), rpy_.z());

  /* reset controller using odom */
  double x[NX] = { pos_.x(), pos_.y(), pos_.z(), vel_.x(), vel_.y(), vel_.z(), q.w(), q.x(), q.y(), q.z() };
  double u[NU] = { 0.0, 0.0, 0.0, gravity_const_ };
  MPC::initPredXU(x_u_ref_);
  for (int i = 0; i < NN; i++)
  {
    std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);

  mpc_solver_.reset(x_u_ref_);
}

nav_msgs::Odometry aerial_robot_control::NMPCController::getOdom()
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

void aerial_robot_control::NMPCController::controlCore()
{
  /* get odom information */
  nav_msgs::Odometry odom_now = getOdom();

  /* set target */
  tf::Vector3 target_pos_ = navigator_->getTargetPos();
  double x[NX] = { target_pos_.x(), target_pos_.y(), target_pos_.z(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
  double u[NU] = { 0.0, 0.0, 0.0, gravity_const_ };

  for (int i = 0; i < NN; i++)
  {
    // shift one step
    std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
              x_u_ref_.x.data.begin() + NX * i);
    std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
              x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);
  std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * NN);

  /* solve */
  mpc_solver_.solve(odom_now, x_u_ref_);

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

  // TODO: should be set by rosparam   0.025 - 40hz for controller; 0.1 - 10hz for predict horizon
  // roll, pitch
  target_roll_ = (0.025 * rpy0[0] + (0.1 - 0.025) * rpy1[0]) / 0.1;
  target_pitch_ = (0.025 * rpy0[1] + (0.1 - 0.025) * rpy1[1]) / 0.1;

  flight_cmd_.angles[0] = static_cast<float>(target_roll_);
  flight_cmd_.angles[1] = static_cast<float>(target_pitch_);

  // yaw
  double target_yaw, error_yaw, yaw_p_term, error_yaw_rate, yaw_d_term;
  target_yaw = (0.025 * rpy0[2] + (0.1 - 0.025) * rpy1[2]) / 0.1;
  error_yaw = angles::shortest_angular_distance(target_yaw, rpy0[2]);
  yaw_p_term = 4 * error_yaw;
  error_yaw_rate = mpc_solver_.x_u_out_.x.data.at(9) - mpc_solver_.x_u_out_.x.data.at(9 + NX);
  yaw_d_term = 0.1 * error_yaw_rate;
  candidate_yaw_term_ = -yaw_p_term - yaw_d_term;

  flight_cmd_.angles[2] = static_cast<float>(candidate_yaw_term_);

  // thrust
  double thrust = mpc_solver_.x_u_out_.u.data.at(3) * mass_;

  std::vector<float> target_base_thrust_;
  target_base_thrust_.resize(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = static_cast<float>(thrust / motor_num_);
  }
  flight_cmd_.base_thrust = target_base_thrust_;
}

void aerial_robot_control::NMPCController::sendCmd()
{
  pub_flight_cmd_.publish(flight_cmd_);
}

void aerial_robot_control::NMPCController::callback_viz(const ros::TimerEvent& event)
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
