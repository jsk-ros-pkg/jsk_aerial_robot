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

  joint_torque_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  joint_yaw_torque_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("joint_yaw/torque_enable", boost::bind(&WalkController::servoTorqueCtrlCallback, this, _1, _2, "yaw"));
  joint_pitch_torque_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("joint_pitch/torque_enable", boost::bind(&WalkController::servoTorqueCtrlCallback, this, _1, _2, "pitch"));

  joint_no_load_sub_ = nh_.subscribe<std_msgs::Empty>("joint_no_load", 1, &WalkController::jointNoLoadCallback, this);
  joint_force_compliance_sub_ = nh_.subscribe<std_msgs::Empty>("joint_force_comliance", 1, &WalkController::jointForceComplianceCallback, this);

  target_joint_state_.name = tiger_robot_model_->getLinkJointNames();
  target_joint_state_.position.resize(target_joint_state_.name.size(), 0);

  force_joint_torque_ = false;
  joint_no_load_end_t_ = 0;
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

  if (navigator_->getNaviState() == aerial_robot_navigation::START_STATE) {
    control_timestamp_ = ros::Time::now().toSec();
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
  auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  Eigen::VectorXd static_joint_torque = tiger_robot_model_->getStaticJointT();

  const auto& names = target_joint_state_.name;
  auto& positions = target_joint_state_.position;
  for(int i = 0; i < names.size(); i++) {

    auto n = names.at(i);
    const auto& v = current_joint_state.name;
    auto res = std::find(v.begin(), v.end(), n);

    if (res == v.end()) {
      ROS_ERROR_STREAM("[Tiger] joint torque compiance control, cannot find " << n);
      continue;
    }

    auto id = std::distance(v.begin(), res);

    double tor = static_joint_torque(i);

    if (joint_no_load_end_t_ > ros::Time::now().toSec()) {

      tor = 0;

      if (joint_no_load_end_t_ - ros::Time::now().toSec() < 0.1) {
        force_joint_torque_ = false;
      }
      else {
        force_joint_torque_ = true;
      }
    }

    double delta_angle = tor / tor_kp_;
    double target_angle = current_joint_state.position.at(id) + delta_angle;
    positions.at(i) = target_angle;
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

  // send joint compliance command
  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE ||
      force_joint_torque_) {

    double st = target_joint_state_.header.stamp.toSec();

    if (ros::Time::now().toSec() - st > 1 / joint_ctrl_rate_) {
      target_joint_state_.header.stamp = ros::Time::now();
      joint_control_pub_.publish(target_joint_state_);
    }
  }

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
}

bool WalkController::servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& name)
{
  int offset = 0;
  if (name == std::string("yaw")) offset = 0;
  if (name == std::string("pitch")) offset = 1;

  int rotor_num = tiger_robot_model_->getRotorNum();
  spinal::ServoTorqueCmd torque_msg;
  for (int i = 0; i < rotor_num; i++) {
    torque_msg.index.push_back(4 * i + offset);
    torque_msg.torque_enable.push_back(req.data);
  }
  joint_torque_pub_.publish(torque_msg);

  if (!req.data) force_joint_torque_ = false;

  return true;
}

void WalkController::jointForceComplianceCallback(const std_msgs::EmptyConstPtr& msg)
{
  force_joint_torque_ = true;
}

void WalkController::jointNoLoadCallback(const std_msgs::EmptyConstPtr& msg)
{
  joint_no_load_end_t_ = ros::Time::now().toSec() + 5.0; // 10.0 is a paramter
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::Tiger::WalkController, aerial_robot_control::ControlBase);
