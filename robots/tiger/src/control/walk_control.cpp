// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <tiger/control/walk_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control::Tiger;

WalkController::WalkController():
  PoseLinearController()
{
}

void WalkController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();

  tiger_robot_model_ = boost::dynamic_pointer_cast<::Tiger::FullVectoringRobotModel>(robot_model);

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);
  target_joint_state_.position.resize(0);
  target_joint_state_.name.resize(0);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
}

void WalkController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "joint_ctrl_rate", joint_ctrl_rate_, 1.0); // 1 Hz

  // calculate the torque_position Kp
  double angle_scale;
  getParam<double>(nh_, "servo_controller/joints/angle_scale", angle_scale, 1.0);
  double torque_load_scale;
  getParam<double>(nh_, "servo_controller/joints/torque_scale", torque_load_scale, 1.0);
  double load_kp;
  getParam<double>(nh_, "servo_controller/joints/load_kp", load_kp, 1.0); // KP / 128
  tor_kp_ = torque_load_scale * load_kp / angle_scale;
}

bool WalkController::update()
{
  ControlBase::update();

  // skip before the model initialization
  if (tiger_robot_model_->getStaticVectoringF().size() == 0 ||
      tiger_robot_model_->getStaticJointT().size() == 0) {
    return false;
  }

  if (target_joint_state_.position.size() == 0) {
    target_joint_state_ = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
    compliance_joint_state_.name = tiger_robot_model_->getLinkJointNames();
    compliance_joint_state_.position.resize(compliance_joint_state_.name.size(), 0);
  }

  if (navigator_->getNaviState() == aerial_robot_navigation::START_STATE) {
    control_timestamp_ = ros::Time::now().toSec();

    target_joint_state_ = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  }

  // only consider the static balance
  Eigen::VectorXd static_thrust_force = tiger_robot_model_->getStaticVectoringF();
  for(int i = 0; i < motor_num_; i++) {
    Eigen::Vector3d f = static_thrust_force.segment(3 * i, 3);

    double lambda = f.norm();
    double roll = atan2(-f.y(), f.z());
    double pitch = atan2(f.x(), -f.y() * sin(roll) + f.z() * cos(roll));

    target_base_thrust_.at(i)= lambda;
    target_gimbal_angles_.at(2 * i) = roll;
    target_gimbal_angles_.at(2 * i + 1) = pitch;
  }
  target_vectoring_f_ = static_thrust_force;

  // consider the compliance of joint control
  Eigen::VectorXd static_joint_torque = tiger_robot_model_->getStaticJointT();

  const auto& names = compliance_joint_state_.name;
  auto& positions = compliance_joint_state_.position;
  for(int i = 0; i < names.size(); i++) {

    auto n = names.at(i);
    const auto& v = target_joint_state_.name;
    auto res = std::find(v.begin(), v.end(), n);

    if (res == v.end()) {
      ROS_ERROR_STREAM("[Tiger] joint torque compiance control, cannot find " << n);
      continue;
    }

    auto id = std::distance(v.begin(), res);

    double tor = static_joint_torque(i);
    double delta_angle = tor / tor_kp_;
    double compliance_angle = target_joint_state_.position.at(id) + delta_angle;
    positions.at(i) = compliance_angle;
  }

  // TODO: add feed-back control
  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {
    PoseLinearController::controlCore();
  }

  // send ros message for monitoring
  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++) {
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);


  // send control command to robot
  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {
    sendCmd();
  }

  return true;
}

void WalkController::sendCmd()
{
  PoseLinearController::sendCmd();

  /* send base throttle command */
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  /* send gimbal control command */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();

  for(int i = 0; i < motor_num_ * 2; i++)
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));

  gimbal_control_pub_.publish(gimbal_control_msg);

  /* send joint control command */
  double st = compliance_joint_state_.header.stamp.toSec();
  if (ros::Time::now().toSec() - st > 1 / joint_ctrl_rate_) {
    compliance_joint_state_.header.stamp = ros::Time::now();
    joint_control_pub_.publish(compliance_joint_state_);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::Tiger::WalkController, aerial_robot_control::ControlBase);
