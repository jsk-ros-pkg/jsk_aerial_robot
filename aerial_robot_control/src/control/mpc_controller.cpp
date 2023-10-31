// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/27.
//

#include "aerial_robot_control/control/mpc_controller.h"

aerial_robot_control::MPCController::MPCController() : ControlBase()
{
}

void aerial_robot_control::MPCController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  mpc_solver_.initialize();

  ros::NodeHandle control_nh(nh_, "controller");

  mass_ = robot_model_->getMass();
  gravity_const_ =
      robot_model_->getGravity()[2];  // TODO: wired. I think the original value should be -9.8 in ENU frame
  //  ROS_ERROR("gravity size: %td", robot_model_->getGravity().size());
  //  ROS_ERROR("gravity: %f, %f, %f", robot_model_->getGravity()[0], robot_model_->getGravity()[1],
  //  robot_model_->getGravity()[2]);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);

  // get these parameters from rosparam
  //  ros::param::get("~has_traj_server", has_traj_server_);
  //  ros::param::get("~has_pred_viz", has_pred_viz_);
  //  ros::param::get("~pred_viz_type", pred_viz_type_);
  //  ros::param::get("~is_build_acados", is_build_acados_);
  //  ros::param::get("~has_pred_pub", has_pred_pub_);

  reset();

  ROS_INFO("MPC Controller initialized!");
}

bool aerial_robot_control::MPCController::update()
{
  if (!ControlBase::update())
    return false;

  /* get odom information */
  nav_msgs::Odometry odom_now = getOdom();

  /* set target */
  tf::Vector3 target_pos_ = navigator_->getTargetPos();
  double x[NX] = { odom_now.pose.pose.position.x,
                   odom_now.pose.pose.position.y,
                   odom_now.pose.pose.position.z,
                   0.0,
                   0.0,
                   0.0,
                   1.0,
                   0.0,
                   0.0,
                   0.0 };
  double u[NU] = { 0.0, 0.0, 0.0, gravity_const_ };
  MPC::initPredXU(x_u_ref_);
  for (int i = 0; i < N; i++)
  {
    std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * N);

  /* solve */
  mpc_solver_.solve(odom_now, x_u_ref_);

  /* get result */
  // convert quaternion to rpy
  tf::Quaternion q;
  q.setW(mpc_solver_.x_u_out_.x.data.at(6));
  q.setX(mpc_solver_.x_u_out_.x.data.at(7));
  q.setY(mpc_solver_.x_u_out_.x.data.at(8));
  q.setZ(mpc_solver_.x_u_out_.x.data.at(9));
  double thrust = mpc_solver_.x_u_out_.u.data.at(0) * mass_;

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  flight_cmd_.angles[0] = static_cast<float>(roll);
  flight_cmd_.angles[1] = static_cast<float>(pitch);
  flight_cmd_.angles[2] = static_cast<float>(yaw);

  std::vector<float> target_base_thrust_;
  target_base_thrust_.resize(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = static_cast<float>(thrust / motor_num_);
  }
  flight_cmd_.base_thrust = target_base_thrust_;

  /* pub */
  flight_cmd_pub_.publish(flight_cmd_);

  // output flight cmd using ROS INFO
  ROS_INFO("r: %f, p: %f, y: %f, 1/4 thrust: %f", flight_cmd_.angles[0], flight_cmd_.angles[1], flight_cmd_.angles[2],
           flight_cmd_.base_thrust[0]);

  return true;
}

void aerial_robot_control::MPCController::reset()
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
  for (int i = 0; i < N; i++)
  {
    std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * N);

  mpc_solver_.reset(x_u_ref_);
}

nav_msgs::Odometry aerial_robot_control::MPCController::getOdom()
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

void aerial_robot_control::MPCController::sendFlightCmd()
{
  /* angle */
  tf::Quaternion q;
  q.setW(mpc_solver_.x_u_out_.x.data.at(6));
  q.setX(mpc_solver_.x_u_out_.x.data.at(7));
  q.setY(mpc_solver_.x_u_out_.x.data.at(8));
  q.setZ(mpc_solver_.x_u_out_.x.data.at(9));

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  flight_cmd_.angles[0] = static_cast<float>(roll);
  flight_cmd_.angles[1] = static_cast<float>(pitch);
  flight_cmd_.angles[2] = static_cast<float>(yaw);

  /* thrust */
  double thrust = mpc_solver_.x_u_out_.u.data.at(0) * mass_;

  std::vector<float> target_base_thrust_;
  target_base_thrust_.resize(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = static_cast<float>(thrust / motor_num_);
  }
  flight_cmd_.base_thrust = target_base_thrust_;

  flight_cmd_pub_.publish(flight_cmd_);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::MPCController, aerial_robot_control::ControlBase);
