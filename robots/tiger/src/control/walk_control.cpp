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
  joint_index_map_(0),
  joint_soft_compliance_(false),
  joint_compliance_end_t_(0),
  prev_navi_target_joint_angles_(0)
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
  tiger_walk_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::Tiger::WalkNavigator>(navigator);

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);
  target_joint_angles_.position.resize(0);
  target_joint_angles_.name.resize(0);
  target_vectoring_f_ = Eigen::VectorXd::Zero(3 * motor_num_);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  joint_angle_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  joint_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_torque_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  link_rot_thrust_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/link_rot_thrust_force", 1);

  joint_servo_enable_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  joint_yaw_torque_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("joint_yaw/torque_enable", boost::bind(&WalkController::servoTorqueCtrlCallback, this, _1, _2, "yaw"));
  joint_pitch_torque_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("joint_pitch/torque_enable", boost::bind(&WalkController::servoTorqueCtrlCallback, this, _1, _2, "pitch"));
  joint_force_compliance_sub_ = nh_.subscribe<std_msgs::Empty>("joint_force_compliance", 1, &WalkController::jointSoftComplianceCallback, this);

  target_joint_angles_.name = tiger_robot_model_->getLinkJointNames();
  int joint_num = tiger_robot_model_->getLinkJointNames().size();
  target_joint_angles_.position.assign(joint_num, 0);

  target_joint_torques_.name.resize(0);
  target_joint_torques_.position.resize(0);

  for (int i = 0; i < motor_num_; i++) {
    fw_i_terms_.push_back(Eigen::Vector3d::Zero());
  }
}

void WalkController::rosParamInit()
{
  ros::NodeHandle walk_control_nh(nh_, "controller/walk");
  getParam<double>(walk_control_nh, "joint_ctrl_rate", joint_ctrl_rate_, 1.0); // 1 Hz
  getParam<double>(walk_control_nh, "joint_torque_control_thresh", joint_torque_control_thresh_, 2.0); // 2 Nm
  getParam<double>(walk_control_nh, "servo_angle_bias", servo_angle_bias_, 0.02); // 0.02 rad
  getParam<double>(walk_control_nh, "servo_max_torque", servo_max_torque_, 6.0); // 6.0 Nm
  getParam<double>(walk_control_nh, "servo_torque_change_rate", servo_torque_change_rate_, 1.5); // rate
  getParam<double>(walk_control_nh, "link_rot_f_control_i_thresh", link_rot_f_control_i_thresh_, 0.06); // rad

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

  std::vector<int> xy_indices = {X, Y};
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(xy_nh));
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, xy_indices));

  loadParam(z_nh);
  walk_pid_controllers_.push_back(PID("z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(z_nh));
  std::vector<int> z_indices = {Z};
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, z_indices));

  // for link-wise rotation control
  ros::NodeHandle link_nh(walk_control_nh, "link");
  loadParam(link_nh);
  walk_pid_controllers_.push_back(PID("link", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(link_nh));
  std::vector<int> link_indices = {Z+1};
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, link_indices));


  // calculate the torque_position Kp
  getParam<double>(nh_, "servo_controller/joints/angle_scale", angle_scale_, 1.0);
  getParam<double>(nh_, "servo_controller/joints/torque_scale", torque_load_scale_, 1.0);
  double load_kp;
  getParam<double>(nh_, "servo_controller/joints/load_kp", load_kp, 1.0); // KP / 128
  tor_kp_ = torque_load_scale_ * load_kp / angle_scale_;
}

bool WalkController::update()
{
  ControlBase::update();

  // skip before the model initialization
  if (tiger_robot_model_->getStaticVectoringF().size() == 0 ||
      tiger_robot_model_->getStaticJointT().size() == 0) {
    return false;
  }

  // set (initialize) joint index map
  if (joint_index_map_.size() == 0) {
    setJointIndexMap();
  }

  if (navigator_->getNaviState() == aerial_robot_navigation::START_STATE) {
    control_timestamp_ = ros::Time::now().toSec();
  }

  // thrust control
  thrustControl();

  // joint control
  jointControl();

  // send control command to robot
  sendCmd();

  return true;
}

void WalkController::thrustControl()
{
  if (navigator_->getNaviState() != aerial_robot_navigation::ARM_ON_STATE) {
    return;
  }

  tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_vel = estimator_->getVel(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

  tf::Vector3 baselink_target_pos = tiger_walk_navigator_->getTargetBaselinkPos();
  tf::Vector3 baselink_target_vel = tiger_walk_navigator_->getTargetBaselinkVel();
  tf::Vector3 baselink_target_rpy = tiger_walk_navigator_->getTargetBaselinkRpy();

  // 1. feed-forwared control: compensate the static balance
  Eigen::VectorXd static_thrust_force = tiger_robot_model_->getStaticVectoringF();
  target_vectoring_f_ = static_thrust_force;
  //ROS_INFO_STREAM("[Tiger] [Control] total thrust vector init: " << target_vectoring_f_.transpose());

  // 2. feed-back control:  baselink position control
  Eigen::VectorXd target_wrench = Eigen::VectorXd::Zero(6);

  tf::Vector3 pos_err = baselink_target_pos - baselink_pos;
  tf::Vector3 vel_err = baselink_target_vel - baselink_vel;
  tf::Vector3 rpy_err = baselink_target_rpy - baselink_rpy;

  // no negative (downward) force along z axis
  if (pos_err.z() < 0) {
    // set pos error in z axis to zero => no downward force
    pos_err.setZ(0);
  }

  // time diff
  double du = ros::Time::now().toSec() - control_timestamp_;

  // x
  walk_pid_controllers_.at(X).update(pos_err.x(), du, vel_err.x());

  // y
  walk_pid_controllers_.at(Y).update(pos_err.y(), du, vel_err.y());

  // z
  walk_pid_controllers_.at(Z).update(pos_err.z(), du, vel_err.z());

  // w.r.t. world frame
  tf::Vector3 target_acc(walk_pid_controllers_.at(X).result(),
                         walk_pid_controllers_.at(Y).result(),
                         walk_pid_controllers_.at(Z).result());

  // assign to target wrench
  target_wrench.head(3) = Eigen::Vector3d(target_acc.x(), target_acc.y(), target_acc.z());


  // ros pub
  pid_msg_.x.total.at(0) =  walk_pid_controllers_.at(X).result();
  pid_msg_.x.p_term.at(0) = walk_pid_controllers_.at(X).getPTerm();
  pid_msg_.x.i_term.at(0) = walk_pid_controllers_.at(X).getITerm();
  pid_msg_.x.d_term.at(0) = walk_pid_controllers_.at(X).getDTerm();
  pid_msg_.x.target_p = baselink_target_pos.x();
  pid_msg_.x.err_p = pos_err.x();
  pid_msg_.x.target_d = baselink_target_vel.x();
  pid_msg_.x.err_d = vel_err.x();

  pid_msg_.y.total.at(0) =  walk_pid_controllers_.at(Y).result();
  pid_msg_.y.p_term.at(0) = walk_pid_controllers_.at(Y).getPTerm();
  pid_msg_.y.i_term.at(0) = walk_pid_controllers_.at(Y).getITerm();
  pid_msg_.y.d_term.at(0) = walk_pid_controllers_.at(Y).getDTerm();
  pid_msg_.y.target_p = baselink_target_pos.y();
  pid_msg_.y.err_p = pos_err.y();
  pid_msg_.y.target_d = baselink_target_vel.y();
  pid_msg_.y.err_d = vel_err.y();

  pid_msg_.z.total.at(0) =  walk_pid_controllers_.at(Z).result();
  pid_msg_.z.p_term.at(0) = walk_pid_controllers_.at(Z).getPTerm();
  pid_msg_.z.i_term.at(0) = walk_pid_controllers_.at(Z).getITerm();
  pid_msg_.z.d_term.at(0) = walk_pid_controllers_.at(Z).getDTerm();
  pid_msg_.z.target_p = baselink_target_pos.z();
  pid_msg_.z.err_p = pos_err.z();
  pid_msg_.z.target_d = baselink_target_vel.z();
  pid_msg_.z.err_d = vel_err.z();

  // update
  control_timestamp_ = ros::Time::now().toSec();

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
  // ROS_INFO_STREAM_THROTTLE(1.0, "[fb control] fb vectoring f: " << fb_vectoring_f.transpose());
  // ROS_INFO_STREAM("[Tiger] [Control] fb vectoring f: " << fb_vectoring_f.transpose());
  // ROS_INFO_STREAM("[Tiger] [Control] total thrust vector after fc center: " << target_vectoring_f_.transpose());


  // 3. feed-back control: link-wise rotation control
  Eigen::VectorXd fb_vectoring_l = Eigen::VectorXd::Zero(3 * motor_num_);
  auto target_link_rots = tiger_walk_navigator_->getTargetLinkRots();
  const auto& seg_tf_map = tiger_robot_model_->getSegmentsTf();
  KDL::Frame fr_baselink = seg_tf_map.at(tiger_robot_model_->getBaselinkName());
  tf::Transform tf_w_baselink(estimator_->getOrientation(Frame::BASELINK, estimate_mode_), baselink_pos);
  KDL::Frame fw_baselink;
  tf::transformTFToKDL(tf_w_baselink, fw_baselink);
  double p_gain = walk_pid_controllers_.back().getPGain();
  double i_gain = walk_pid_controllers_.back().getIGain();
  double limit_sum = walk_pid_controllers_.back().getLimitSum();
  double limit_p = walk_pid_controllers_.back().getLimitP();
  double limit_i = walk_pid_controllers_.back().getLimitI();

  // ROS_INFO_STREAM("p gain: " << p_gain << "; i gain: " << i_gain << "; limit sum: " << limit_sum << "; limit p: " << limit_p << "; limit i: " << limit_i);
  std::stringstream ss;
  for(int i = 0; i < motor_num_; i++) {
    std::string link_name = std::string("link") + std::to_string(i+1);
    KDL::Frame fr_link = seg_tf_map.at(link_name);
    KDL::Frame fb_link = fr_baselink.Inverse() * fr_link;
    KDL::Frame fw_link = fw_baselink * fb_link;

    // calculate the control vector from
    // negative means the pivot point is the leg end point
    int direct = -1;

    if (i / 2 == tiger_robot_model_->getFreeleg()) {
      // no contorl for free leg which may induce unstability
      continue;
    }

    if (i % 2 == 1) {
      // outer link (e.g., link2)
      // no control for outer link since the effector is too small
      continue;
    }

    Eigen::Vector3d a = direct * aerial_robot_model::kdlToEigen(fw_link.M.UnitX());
    Eigen::Vector3d b = direct * aerial_robot_model::kdlToEigen(target_link_rots.at(i).UnitX());
    Eigen::Vector3d c = a.cross(b);
    Eigen::Vector3d d = c.cross(a);
    Eigen::Vector3d d_temp = b - a;
    double theta = asin(c.norm());

    // PI control
    // P term
    Eigen::Vector3d fw_p_term = clamp(p_gain * d, limit_p);

    // I term
    if (fabs(theta) > link_rot_f_control_i_thresh_) {
      fw_i_terms_.at(i) = clamp(fw_i_terms_.at(i) + i_gain * d, limit_i);
    }

    Eigen::Vector3d fw = clamp(fw_p_term + fw_i_terms_.at(i), limit_sum);
    Eigen::Vector3d fb = aerial_robot_model::kdlToEigen(fw_link.M.Inverse()) * fw;

    if (i % 2 == 0) {
      // inner link (e.g., link1)
      // no downward force in world frame
      if (fb.z() < 0) fb.z() = 0;
    }

    fb_vectoring_l.segment(3 * i, 3) = fb;

    // ROS_INFO_STREAM("link" << i+1 << ", a: " << a.transpose() << ", b:" << b.transpose() << ", d: " << d.transpose() << ", d_temp: " << d_temp.transpose() << ", fb " << fb.transpose() << ", theta: " << theta);
  }
  target_vectoring_f_ += fb_vectoring_l;
  //ROS_INFO_STREAM_THROTTLE(1.0, "[Tiger][Control] thrust control for link-wise rotation, thrust vector: " << fb_vectoring_l.transpose());
  //ROS_INFO_STREAM_THROTTLE(1.0, "[Tiger] [Control] [Link Rot] theta: " << ss.str());
  // ROS_INFO_STREAM("[Tiger][Control] thrust control for link-wise rotation, thrust vector: " << fb_vectoring_l.transpose());
  // ROS_INFO_STREAM("[Tiger] [Control] [Link Rot] theta: " << ss.str());
  // ROS_INFO_STREAM("[Tiger] [Control] total thrust vector after fc link: " << target_vectoring_f_.transpose());

  std_msgs::Float32MultiArray msg;
  for(int i = 0; i < fb_vectoring_l.size(); i++) {
    msg.data.push_back(fb_vectoring_l(i));
  }
  link_rot_thrust_force_pub_.publish(msg);


  // 4. target lambda and gimbal angles
  for(int i = 0; i < motor_num_; i++) {
    Eigen::Vector3d f = target_vectoring_f_.segment(3 * i, 3);

    double lambda = f.norm();
    double roll = atan2(-f.y(), f.z());
    double pitch = atan2(f.x(), -f.y() * sin(roll) + f.z() * cos(roll));

    target_base_thrust_.at(i)= lambda;
    target_gimbal_angles_.at(2 * i) = roll;
    target_gimbal_angles_.at(2 * i + 1) = pitch;
  }
}

void WalkController::jointControl()
{
  // reset
  target_joint_torques_.name.resize(0);
  target_joint_torques_.effort.resize(0);

  // do joint soft compliance control
  if (joint_soft_compliance_) {
    jointSoftComplianceControl();
    return;
  }

  // basically, use position control for all joints
  auto navi_target_joint_angles = tiger_walk_navigator_->getTargetJointState().position;
  target_joint_angles_.position = navi_target_joint_angles;
  if (prev_navi_target_joint_angles_.size() == 0) {
    prev_navi_target_joint_angles_ = navi_target_joint_angles;
  }
  if (navigator_->getNaviState() == aerial_robot_navigation::START_STATE) {
    prev_navi_target_joint_angles_ = navi_target_joint_angles;
  }

  // std::stringstream ss1, ss2;
  // for (int i = 0; i < prev_navi_target_joint_angles_.size(); i++) {
  //   ss1 << prev_navi_target_joint_angles_.at(i) << ", ";
  //   ss2 << navi_target_joint_angles.at(i) << ", ";
  // }
  //ROS_INFO_STREAM("prev_navi_target_joint_angles: " << ss1.str());
  //ROS_INFO_STREAM("navi_target_joint_angles: " << ss2.str());

  auto current_angles = getCurrentJointAngles();
  if (current_angles.size() == 0 || navi_target_joint_angles.size() == 0) {
    return;
  }

  // use torque control for joints that needs large torque load
  Eigen::VectorXd static_joint_torque = tiger_robot_model_->getStaticJointT();
  const auto& names = target_joint_angles_.name;
  auto& target_angles = target_joint_angles_.position;
  for(int i = 0; i < names.size(); i++) {

    double current_angle = current_angles.at(i);
    double navi_target_angle  = navi_target_joint_angles.at(i);
    double prev_navi_target_angle  = prev_navi_target_joint_angles_.at(i);
    double tor = static_joint_torque(i);
    std::string name = names.at(i);
    int j = atoi(name.substr(5,1).c_str()) - 1; // start from 0

    // position control for small joint torque
    if (fabs(tor) < joint_torque_control_thresh_) {
      continue;
    }

    // position control for yaw joints
    if (name.find("yaw") != std::string::npos) {
      // ROS_INFO_STREAM("position control for: " << name);
      continue;
    }

    // heuristic rule for joints in free leg
    if (j / 2 == tiger_robot_model_->getFreeleg()) {

      prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);

      if (j % 2 == 0) {
        // inner joint (e.g., joint1_pitch)
        // torque rule: set the torque bound as the static torque (small value) for raise and max torque for lower.
        //              joint is expected to raise / lower quick,
        // angle rule: basic position control

        // set joint torque
        target_joint_torques_.name.push_back(name);
        if (tiger_walk_navigator_->getLowerLegFlag()) {
          tor = servo_max_torque_; // largest torque to lower leg
        }
        target_joint_torques_.effort.push_back(tor);

        ROS_INFO_STREAM(name << ", free leg mode, use static torque:" << tor  << "; target angle: " << target_angles.at(i) << "; current angle: " << current_angle);
      }

      if (j % 2 == 1) {
        // outer joint (e.g., joint2_pitch)
        // torque rule: set the largest one
        // angle rule: basic position control

        // set joint torque
        target_joint_torques_.name.push_back(name);
        target_joint_torques_.effort.push_back(servo_max_torque_);

        ROS_INFO_STREAM(name << ", free leg mode, use max torque:" << servo_max_torque_ << "; target angle: " << target_angles.at(i) << "; current angle: " << current_angle);
      }

      continue;
    }

    // set joint angles
    // large diff from real target angles to reach the target joint torque: torque control
    double extra_angle_err = 0.1; // for enough margin for large angle error
    target_angles.at(i) += (tor / fabs(tor) * extra_angle_err);

    // Note: tor > 0: inner pitch joint (e.g., joint1_pitch)
    //       tor < 0: outer pitch joint (e.g., joint2_pitch)
    if (fabs(prev_navi_target_angle - navi_target_angle) > 1e-4) {

      // address the angle bias in pulley system
      double modified_navi_target_angle = navi_target_angle +  tor / fabs(tor) * servo_angle_bias_;

      if (navi_target_angle > prev_navi_target_angle) {
        if (current_angle >= modified_navi_target_angle) {
          // the joint converges
          prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);
          ROS_INFO_STREAM("[Tiger][Control]" << name << " reaches the new target angle " << navi_target_angle);
        }
        else {
          // increase servo torque to reach the target angle, feedforwardly
          if (tor > 0) {
            tor = clamp(tor * servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << name << " increase torque.");
          }
          else {
            // increase servo torque to reach the target angle, feedforwardly
            tor = clamp(tor / servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << name << " decrease torque.");

            // set joint angles
            // set to the current angle for small joint torque
            target_angles.at(i) = current_angle;
          }
        }
      }

      if (navi_target_angle < prev_navi_target_angle) {
        if (current_angle <= modified_navi_target_angle) {
          // the joint converges
          prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);
          ROS_INFO_STREAM("[Tiger][Control]" << name << " reaches the new target angle " << navi_target_angle);
        }
        else {

          if (tor > 0) {
            // decrease servo torque to reach the target angle, feedforwardly
            tor = clamp(tor / servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << name << " decrease torque.");

            // set joint angles
            // set to the current angle for small joint torque
            target_angles.at(i) = current_angle;
          }
          else {
            // increase servo torque to reach the target angle, feedforwardly
            tor = clamp(tor * servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << name << " increase torque.");
          }
        }
      }

      ROS_WARN("[Tiger][Control] %s not converge. prev target: %f, curr target: %f, modified target: %f, real target: %f, curr pos: %f, tor: %f",
               name.c_str(), prev_navi_target_angle, navi_target_angle,
               modified_navi_target_angle, target_angles.at(i), current_angle, tor);
    }

    // set joint torque
    target_joint_torques_.name.push_back(names.at(i));
    target_joint_torques_.effort.push_back(tor);
  }
}

void WalkController::jointSoftComplianceControl()
{
  // use torque control for pitch joints that needs large torque load
  const auto current_angles = getCurrentJointAngles();

  if (current_angles.size() == 0) {
    return;
  }

  if (joint_compliance_end_t_ < ros::Time::now().toSec()) {
    joint_soft_compliance_ = false;
  }

  // compliance to fit the current joint angles
  target_joint_angles_.position = current_angles;
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

  // send joint command
  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE ||
      joint_soft_compliance_) {

    // joint target angles
    double st = target_joint_angles_.header.stamp.toSec();
    if (ros::Time::now().toSec() - st > 1 / joint_ctrl_rate_) {
      target_joint_angles_.header.stamp = ros::Time::now();
      joint_angle_pub_.publish(target_joint_angles_);

      // joint target torques
      if (target_joint_torques_.name.size() > 0) {
        joint_torque_pub_.publish(target_joint_torques_);
      }
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
  spinal::ServoTorqueCmd servo_enable_msg;
  for (int i = 0; i < rotor_num; i++) {
    servo_enable_msg.index.push_back(4 * i + offset);
    servo_enable_msg.torque_enable.push_back(req.data);
  }
  joint_servo_enable_pub_.publish(servo_enable_msg);

  return true;
}

void WalkController::jointSoftComplianceCallback(const std_msgs::EmptyConstPtr& msg)
{
  joint_soft_compliance_ = true;
  joint_compliance_end_t_ = ros::Time::now().toSec() + 5.0; // 10.0 is a paramter
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


// utils
void WalkController::setJointIndexMap()
{
  const auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  const auto& search_v = current_joint_state.name;

  joint_index_map_.resize(0); // resize
  const auto& joint_names = target_joint_angles_.name;
  for(const auto& n: joint_names) {

    auto res = std::find(search_v.begin(), search_v.end(), n);

    if (res == search_v.end()) {
      ROS_ERROR_STREAM("[Tiger] joint index mapping, cannot find " << n);
      continue;
    }

    auto id = std::distance(search_v.begin(), res);

    joint_index_map_.push_back(id);
  }
}

std::vector<double> WalkController::getCurrentJointAngles()
{
  const auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  std::vector<double> angles(0);
  for(const auto id: joint_index_map_) {
    angles.push_back(current_joint_state.position.at(id));
  }

  return angles;
}

bool WalkController::samejointAngles(std::vector<double> group_a, std::vector<double> group_b)
{
  bool res = true;

  if (group_a.size() != group_b.size()) {
    ROS_ERROR_STREAM("[Tiger][Control] compare joint angles between two groups, but the sizes are different. " <<  group_a.size() << "VS " << group_b.size());
    return false;
  }

  for (int i = 0; i < group_a.size(); i++) {
    if (group_a.at(i) != group_b.at(i)) {
        res = false;
        break;
      }
  }

  return res;
}

void WalkController::reset()
{
  PoseLinearController::reset();
  prev_navi_target_joint_angles_.resize(0);

  for (int i = 0; i < motor_num_; i++) {
    fw_i_terms_.push_back(Eigen::Vector3d::Zero());
  }
}

Eigen::VectorXd WalkController::clamp(Eigen::VectorXd v, double b)
{
  if (b < 0) {
      ROS_ERROR("vector clamp: b (%f) should not be negative", b);
      return v;
    }

  if (v.norm() <= b) return v;

  return v.normalized() * b;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::Tiger::WalkController, aerial_robot_control::ControlBase);
