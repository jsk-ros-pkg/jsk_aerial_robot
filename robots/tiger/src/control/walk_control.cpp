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
  PoseLinearController(),
  walk_pid_controllers_(0),
  baselink_target_pos_(0,0,0),
  baselink_target_rpy_(0,0,0),
  force_joint_torque_(false),
  joint_no_load_end_t_(0)
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
  target_vectoring_f_ = Eigen::VectorXd::Zero(3 * motor_num_);

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
}

void WalkController::rosParamInit()
{
  ros::NodeHandle walk_control_nh(nh_, "controller/walk");
  getParam<double>(walk_control_nh, "joint_ctrl_rate", joint_ctrl_rate_, 1.0); // 1 Hz

  ros::NodeHandle xy_nh(walk_control_nh, "xy");
  ros::NodeHandle z_nh(walk_control_nh, "z");

  double limit_sum, limit_p, limit_i, limit_d;
  double limit_err_p, limit_err_i, limit_err_d;
  double p_gain, i_gain, d_gain;

  auto loadParam = [&, this](ros::NodeHandle nh)
    {
      getParam<double>(nh, "limit_sum", limit_sum, 1.0e6);
      getParam<double>(nh, "limit_p", limit_p, 1.0e6);
      getParam<double>(nh, "limit_i", limit_i, 1.0e6);
      getParam<double>(nh, "limit_d", limit_d, 1.0e6);
      getParam<double>(nh, "limit_err_p", limit_err_p, 1.0e6);
      getParam<double>(nh, "limit_err_i", limit_err_i, 1.0e6);
      getParam<double>(nh, "limit_err_d", limit_err_d, 1.0e6);

      getParam<double>(nh, "p_gain", p_gain, 0.0);
      getParam<double>(nh, "i_gain", i_gain, 0.0);
      getParam<double>(nh, "d_gain", d_gain, 0.0);
    };

  loadParam(xy_nh);
  walk_pid_controllers_.push_back(PID("x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  walk_pid_controllers_.push_back(PID("y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

  std::vector<int> indices = {X, Y};
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(xy_nh));
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, indices));

  loadParam(z_nh);
  walk_pid_controllers_.push_back(PID("z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(z_nh));
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, std::vector<int>(1, Z)));

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

    // set the target position for baselink
    ROS_INFO("[Walk] set initial position as target position");
    baselink_target_pos_ = estimator_->getPos(Frame::BASELINK, estimate_mode_);
    baselink_target_rpy_ = estimator_->getEuler(Frame::BASELINK, estimate_mode_);
  }


  // feed-forwared control: compensate the static balance
  Eigen::VectorXd static_thrust_force = tiger_robot_model_->getStaticVectoringF();
  target_vectoring_f_ = static_thrust_force;


  // feed-back control:  baselink position control
  Eigen::VectorXd target_wrench = Eigen::VectorXd::Zero(6);

  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {

    // PoseLinearController::controlCore(); // TODO: no need?

    tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
    tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

    tf::Vector3 pos_err = baselink_target_pos_ - baselink_pos;
    tf::Vector3 rpy_err = baselink_target_rpy_ - baselink_rpy;

    // time diff
    double du = ros::Time::now().toSec() - control_timestamp_;

    // z
    walk_pid_controllers_.at(Z).update(pos_err.z(), du, 0);

    // w.r.t. world frame
    tf::Vector3 target_acc(walk_pid_controllers_.at(X).result(),
                           walk_pid_controllers_.at(Y).result(),
                           walk_pid_controllers_.at(Z).result());

    // assign to target wrench
    target_wrench.head(3) = Eigen::Vector3d(target_acc.x(), target_acc.y(), target_acc.z());


    // ros pub
    pid_msg_.z.total.at(0) =  walk_pid_controllers_.at(Z).result();
    pid_msg_.z.p_term.at(0) = walk_pid_controllers_.at(Z).getPTerm();
    pid_msg_.z.i_term.at(0) = walk_pid_controllers_.at(Z).getITerm();
    pid_msg_.z.d_term.at(0) = walk_pid_controllers_.at(Z).getDTerm();
    pid_msg_.z.target_p = baselink_target_pos_.z();
    pid_msg_.z.err_p = pos_err.z();
    pid_msg_.z.target_d = 0;
    pid_msg_.z.err_d = 0;
    ROS_INFO_STREAM_THROTTLE(1.0, "[fb control] z control: " << walk_pid_controllers_.at(Z).result()
                             << "; pos err: " << pos_err.z()
                             << "; P term: " << walk_pid_controllers_.at(Z).getPTerm()
                             << "; I term: " << walk_pid_controllers_.at(Z).getITerm());

    // update
    control_timestamp_ = ros::Time::now().toSec();
  }

  // allocation
  // use all vecotoring angles
  // consider cog and baselink are all level
  const auto rotors_origin = tiger_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto links_rot = tiger_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();

  // Note: only consider the inside links
  Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_ / 2);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  for(int i = 0; i < motor_num_ / 2; i++) {
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin.at(2 * i));
      q_mat.middleCols(3 * i, 3) = wrench_map * links_rot.at(2 * i);
    }
  auto q_mat_inv = aerial_robot_model::pseudoinverse(q_mat);
  Eigen::VectorXd fb_vectoring_f = q_mat_inv * target_wrench; // feed-back control
  for(int i = 0; i < motor_num_ / 2; i++) {
    target_vectoring_f_.segment(3 * 2 * i, 3) += fb_vectoring_f.segment(3 * i, 3);
  }
  ROS_INFO_STREAM_THROTTLE(1.0, "[fb control] fb vectoring f: " << fb_vectoring_f.transpose());


  // target lambda and gimbal angles
  for(int i = 0; i < motor_num_; i++) {
    Eigen::Vector3d f = target_vectoring_f_.segment(3 * i, 3);

    double lambda = f.norm();
    double roll = atan2(-f.y(), f.z());
    double pitch = atan2(f.x(), -f.y() * sin(roll) + f.z() * cos(roll));

    target_base_thrust_.at(i)= lambda;
    target_gimbal_angles_.at(2 * i) = roll;
    target_gimbal_angles_.at(2 * i + 1) = pitch;
  }


  // joint control compliance
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

  // send control command to robot
  sendCmd();

  return true;
}

void WalkController::sendCmd()
{
  PoseLinearController::sendCmd();

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

  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {

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

void WalkController::cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if(config.pid_control_flag)
    {
      switch(level)
        {
        case Levels::RECONFIGURE_P_GAIN:
          for(const auto& index: controller_indices)
            {
              walk_pid_controllers_.at(index).setPGain(config.p_gain);
              ROS_INFO_STREAM("change p gain for walk PID controller '" << walk_pid_controllers_.at(index).getName() << "'");
            }
          break;
        case Levels::RECONFIGURE_I_GAIN:
          for(const auto& index: controller_indices)
            {
              walk_pid_controllers_.at(index).setIGain(config.i_gain);
              ROS_INFO_STREAM("change i gain for walk PID controller '" << walk_pid_controllers_.at(index).getName() << "'");
            }
          break;
        case Levels::RECONFIGURE_D_GAIN:
          for(const auto& index: controller_indices)
            {
              walk_pid_controllers_.at(index).setDGain(config.d_gain);
              ROS_INFO_STREAM("change d gain for walk PID controller '" << walk_pid_controllers_.at(index).getName() << "'");
            }
          break;
        default :
          break;
        }
    }
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::Tiger::WalkController, aerial_robot_control::ControlBase);
