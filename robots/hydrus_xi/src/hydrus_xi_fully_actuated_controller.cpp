// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#include <hydrus_xi/hydrus_xi_fully_actuated_controller.h>

using namespace std;

namespace control_plugin
{
  HydrusXiFullyActuatedController::HydrusXiFullyActuatedController():
    ControlBase(),
    state_pos_(0, 0, 0),
    state_vel_(0, 0, 0),
    target_pos_(0, 0, 0),
    target_vel_(0, 0, 0),
    target_acc_(0, 0, 0),
    pos_err_(0, 0, 0),
    vel_err_(0, 0, 0),
    state_yaw_(0),
    state_yaw_vel_(0),
    target_yaw_(0),
    target_yaw_vel_(0),
    yaw_err_(0),
    xy_i_term_(0, 0, 0),
    start_rp_integration_(false),
    yaw_i_term_(0),
    alt_i_term_(0),
    target_yaw_acc_(0),
    wrench_allocation_matrix_pub_stamp_(ros::Time::now()),
    torque_allocation_matrix_inv_pub_stamp_(ros::Time::now())
  {
  }

  void HydrusXiFullyActuatedController::initialize(ros::NodeHandle nh,
                                                   ros::NodeHandle nhp,
                                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                   double ctrl_loop_rate)
  {
    ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    rosParamInit();

    motor_num_ = robot_model_->getRotorNum();

    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    pid_pub_ = nh_.advertise<aerial_robot_msgs::FlatnessPid>("debug/pos_yaw/pid", 1);

    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
    wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
    wrench_allocation_matrix_inv_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix_inv", 1);

    server_ = boost::make_shared<dynamic_reconfigure::Server<hydrus_xi::FullyActuatedControllerGainsConfig> >(ros::NodeHandle(nh, "gina/generator/att"));
    dynamic_reconf_func_ = boost::bind(&HydrusXiFullyActuatedController::controllerGainsCfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);
  }

  bool HydrusXiFullyActuatedController::update()
  {
    if(!ControlBase::update()) return false;

    stateError();
    pidUpdate();
    sendCmd();
  }

  void HydrusXiFullyActuatedController::reset()
  {
    ControlBase::reset();
    start_rp_integration_ = false;
    xy_i_term_.setZero();
    alt_i_term_ = 0.0;
    yaw_i_term_ = 0.0;
    target_yaw_acc_ = 0.0;
    setAttitudeGains();
  }

  void HydrusXiFullyActuatedController::stateError()
  {
    state_pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
    state_vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
    target_vel_ = navigator_->getTargetVel();
    target_pos_ = navigator_->getTargetPos();
    target_acc_ = navigator_->getTargetAcc();

    state_yaw_ = estimator_->getState(State::YAW_COG, estimate_mode_)[0];
    state_yaw_vel_ = estimator_->getState(State::YAW_COG, estimate_mode_)[1];
    target_yaw_ = navigator_->getTargetYaw();
    target_yaw_vel_ = navigator_->getTargetYawVel();
    yaw_err_ = angles::shortest_angular_distance(state_yaw_, target_yaw_);
  }

  void HydrusXiFullyActuatedController::pidUpdate()
  {
    aerial_robot_msgs::FlatnessPid pid_msg;
    pid_msg.header.stamp = ros::Time::now();

    /* roll/pitch integration flag */
    if(!start_rp_integration_)
      {
        if(state_pos_.z() - estimator_->getLandingHeight() > start_rp_integration_height_)
          {
            start_rp_integration_ = true;
            spinal::FlightConfigCmd flight_config_cmd;
            flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
            navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
            ROS_WARN("start rp integration");
          }
      }

    /* time diff */
    double du = ros::Time::now().toSec() - control_timestamp_;

    /* xy */
    tf::Vector3 xy_p_term, xy_d_term;
    tf::Matrix3x3 uav_rot;
    uav_rot.setRPY(estimator_->getState(State::ROLL_COG, estimate_mode_)[0],
                   estimator_->getState(State::PITCH_COG, estimate_mode_)[0],
                   estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
    tf::Matrix3x3 uav_rot_inv = uav_rot.inverse();

    /* convert from world frame to CoG frame */
    pos_err_ = uav_rot_inv * (target_pos_ - state_pos_);

    switch(navigator_->getXyControlMode())
      {
      case aerial_robot_navigation::POS_CONTROL_MODE:
        {
          /* P */
          xy_p_term = clampV(xy_gains_[0] * pos_err_, -xy_terms_limits_[0], xy_terms_limits_[0]);

          /* I */
          if (start_rp_integration_) {
            xy_i_term_ += (pos_err_ * du * xy_gains_[1]);
            xy_i_term_ = clampV(xy_i_term_, -xy_terms_limits_[1], xy_terms_limits_[1]);
          }

          /* D */
          vel_err_ = uav_rot_inv * (-state_vel_);
          xy_d_term = clampV(xy_gains_[2] * vel_err_, -xy_terms_limits_[2], xy_terms_limits_[2]);
          break;
        }
      case aerial_robot_navigation::VEL_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */
          vel_err_ = uav_rot_inv * (target_vel_ - state_vel_);

          /* P */
          xy_p_term = clampV(xy_gains_[0] * vel_err_, -xy_terms_limits_[0], xy_terms_limits_[0]);
          xy_i_term_.setZero();
          xy_d_term.setZero();
          break;
        }
      case aerial_robot_navigation::ACC_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */

          xy_p_term = uav_rot_inv * target_acc_;
          xy_i_term_.setZero();
          xy_d_term.setZero();
          break;
        }
      default:
        {
          break;
        }
      }

    tf::Vector3 xy_total_term = xy_p_term + xy_i_term_ + xy_d_term + xy_offset_;
    target_linear_acc_.setX(clamp(xy_total_term.x(), -xy_limit_, xy_limit_));
    target_linear_acc_.setY(clamp(xy_total_term.y(), -xy_limit_, xy_limit_));

    /* ros pub */
    //pitch(x)
    pid_msg.pitch.total.push_back(target_linear_acc_.x());
    pid_msg.pitch.p_term.push_back(xy_p_term.x());
    pid_msg.pitch.i_term.push_back(xy_i_term_.x());
    pid_msg.pitch.d_term.push_back(xy_d_term.x());
    pid_msg.pitch.target_pos = target_pos_.x();
    pid_msg.pitch.pos_err = pos_err_.x();
    pid_msg.pitch.target_vel = target_vel_.x();
    pid_msg.pitch.vel_err = vel_err_.x();

    //roll(y)
    pid_msg.roll.total.push_back(target_linear_acc_.y());
    pid_msg.roll.p_term.push_back(xy_p_term.y());
    pid_msg.roll.i_term.push_back(xy_i_term_.y());
    pid_msg.roll.d_term.push_back(xy_d_term.y());
    pid_msg.roll.target_pos = target_pos_.y();
    pid_msg.roll.pos_err = pos_err_.y();
    pid_msg.roll.target_vel = target_vel_.y();
    pid_msg.roll.vel_err = vel_err_.y();

    /* yaw */
    double yaw_err = clamp(yaw_err_, -yaw_err_thresh_, yaw_err_thresh_);
    /* P */
    double yaw_p_term = clamp(yaw_gains_[0] * yaw_err, -yaw_terms_limits_[0], yaw_terms_limits_[0]);
    /* I */
    yaw_i_term_ += (yaw_err_ * du * yaw_gains_[1]);
    yaw_i_term_ = clamp(yaw_i_term_, -yaw_terms_limits_[1], yaw_terms_limits_[1]);
    /* D : usually it is in the flight board (false)*/
    double yaw_d_term = 0;
    if(need_yaw_d_control_)
      yaw_d_term = clamp(yaw_gains_[2] * (-state_yaw_vel_), -yaw_terms_limits_[2], yaw_terms_limits_[2]);

    target_yaw_acc_ = clamp(yaw_p_term + yaw_i_term_ + yaw_d_term, -yaw_limit_, yaw_limit_);
    /* ros pub */
    pid_msg.yaw.total.push_back(target_yaw_acc_);
    pid_msg.yaw.p_term.push_back(yaw_p_term);
    pid_msg.yaw.i_term.push_back(yaw_i_term_);
    pid_msg.yaw.d_term.push_back(yaw_d_term);
    pid_msg.yaw.target_pos = target_yaw_;
    pid_msg.yaw.pos_err = yaw_err_;
    pid_msg.yaw.target_vel = target_yaw_vel_;
    pid_msg.yaw.vel_err = state_yaw_vel_;

    /* alt */
    double alt_err = clamp(pos_err_.z(), -alt_err_thresh_, alt_err_thresh_);

    if(navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE) alt_err += alt_landing_const_i_ctrl_thresh_;

    /* P */
    double alt_p_term = clamp(alt_gains_[0] * alt_err, -alt_terms_limits_[0], alt_terms_limits_[0]);
    if(navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE) alt_p_term = 0;

    /* I */
    if(navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
      alt_i_term_ += alt_err * du * alt_takeoff_i_gain_;
    else
      alt_i_term_ += alt_err * du * alt_gains_[1];

    alt_i_term_ = clamp(alt_i_term_, -alt_terms_limits_[1], alt_terms_limits_[1]);
    /* D */
    double alt_d_term = clamp(alt_gains_[2] * -state_vel_.z(), -alt_terms_limits_[2], alt_terms_limits_[2]);

    target_linear_acc_.setZ(clamp(alt_p_term + alt_i_term_ + alt_d_term + alt_offset_, -alt_limit_, alt_limit_));

    /* ros pub */
    //z(z)
    pid_msg.z.total.push_back(target_linear_acc_.z());
    pid_msg.z.p_term.push_back(alt_p_term);
    pid_msg.z.i_term.push_back(alt_i_term_);
    pid_msg.z.d_term.push_back(alt_d_term);
    pid_msg.z.target_pos = target_pos_.z();
    pid_msg.z.pos_err = alt_err;
    pid_msg.z.target_vel = target_vel_.z();
    pid_msg.z.vel_err = state_vel_.z();

    /* ros publish */
    pid_pub_.publish(pid_msg);

    /* update */
    control_timestamp_ = ros::Time::now().toSec();
  }

  void HydrusXiFullyActuatedController::sendCmd()
  {
    //wrench allocation matrix
    auto Q = calcWrenchAllocationMatrixWithInertial();
    auto Q_inv = aerial_robot_model::pseudoinverse(Q);

    if (verbose_)
      ROS_WARN("det(Q):%e", std::sqrt((Q * Q.transpose()).determinant()));

    if (ros::Time::now().toSec() - wrench_allocation_matrix_pub_stamp_.toSec() > wrench_allocation_matrix_pub_interval_)
      {
        wrench_allocation_matrix_pub_stamp_ = ros::Time::now();
        aerial_robot_msgs::WrenchAllocationMatrix msg;
        msg.f_x.resize(motor_num_);
        msg.f_y.resize(motor_num_);
        msg.f_z.resize(motor_num_);
        msg.t_x.resize(motor_num_);
        msg.t_y.resize(motor_num_);
        msg.t_z.resize(motor_num_);

        for (unsigned int i = 0; i < motor_num_; i++) {
          msg.f_x.at(i) = Q(0, i);
          msg.f_y.at(i) = Q(1, i);
          msg.f_z.at(i) = Q(2, i);
          msg.t_x.at(i) = Q(3, i);
          msg.t_y.at(i) = Q(4, i);
          msg.t_z.at(i) = Q(5, i);
        }

        wrench_allocation_matrix_pub_.publish(msg);

        for (unsigned int i = 0; i < motor_num_; i++) {
          msg.f_x.at(i) = Q_inv(i, 0);
          msg.f_y.at(i) = Q_inv(i, 1);
          msg.f_z.at(i) = Q_inv(i, 2);
          msg.t_x.at(i) = Q_inv(i, 3);
          msg.t_y.at(i) = Q_inv(i, 4);
          msg.t_z.at(i) = Q_inv(i, 5);
        }

        wrench_allocation_matrix_inv_pub_.publish(msg);
      }

    //Simple PID based position/attitude/altitude control
    //send flight command
    spinal::FourAxisCommand flight_command_data;
    double max_yaw_d_gain = 0; // for reconstruct yaw control term in spinal
    for (unsigned int i = 0; i < motor_num_; i++)
      {
        if(Q_inv(i, 5) > max_yaw_d_gain) max_yaw_d_gain = Q_inv(i, 5);
      }
    flight_command_data.angles[2] = target_yaw_acc_ * max_yaw_d_gain;
    flight_command_data.base_throttle = calcForceVector(Q_inv);
    flight_cmd_pub_.publish(flight_command_data);

    //send torque allocation matrix inv
    if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_.toSec() > torque_allocation_matrix_inv_pub_interval_)
      {
        torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now();

        spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
        torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
        Eigen::MatrixXd torque_allocation_matrix_inv = Q_inv.block(0, 3, motor_num_, 3);
        if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
          ROS_ERROR("Torque Allocation Matrix overflow");
        for (unsigned int i = 0; i < motor_num_; i++)
          {
            torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
          }
        torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
      }
  }

  Eigen::MatrixXd HydrusXiFullyActuatedController::calcWrenchAllocationMatrixWithInertial()
  {
    const std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();

    //Q : WrenchAllocationMatrix
    Eigen::MatrixXd Q(6, motor_num_);
    double uav_mass_inv = 1.0 / robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
    for (unsigned int i = 0; i < motor_num_; ++i) {
      Q.block(0, i, 3, 1) = rotors_normal.at(i) * uav_mass_inv;
      Q.block(3, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)));
    }

    return Q;
  }

  std::vector<float> HydrusXiFullyActuatedController::calcForceVector(const Eigen::MatrixXd& wrench_allocation_matrix_inv)
  {
    Eigen::Vector3d target_linear_acc_eigen;
    tf::vectorTFToEigen(target_linear_acc_, target_linear_acc_eigen);
    Eigen::VectorXd propeller_force = wrench_allocation_matrix_inv.block(0, 0, motor_num_, 3) * target_linear_acc_eigen;
    std::vector<float> force(motor_num_);
    for (unsigned int i = 0; i < motor_num_; i++)
      force.at(i) = propeller_force[i];
    return force;
  }

  void HydrusXiFullyActuatedController::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle xy_nh(control_nh, "xy");
    ros::NodeHandle z_nh(control_nh, "z");
    ros::NodeHandle yaw_nh(control_nh, "yaw");
    ros::NodeHandle roll_pitch_nh(control_nh, "roll_pitch");

    getParam<bool>(control_nh, "need_yaw_d_control", need_yaw_d_control_, false);
    getParam<bool>(control_nh, "verbose", verbose_, false);
    getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
    getParam<double>(control_nh, "wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);

    /* z */
    getParam<double>(z_nh, "alt_landing_const_i_ctrl_thresh", alt_landing_const_i_ctrl_thresh_, 0.0);
    getParam<double>(z_nh, "offset", alt_offset_, 0.0);
    getParam<double>(z_nh, "limit", alt_limit_, 1.0e6);
    getParam<double>(z_nh, "p_term_limit", alt_terms_limits_[0], 1.0e6);
    getParam<double>(z_nh, "i_term_limit", alt_terms_limits_[1], 1.0e6);
    getParam<double>(z_nh, "d_term_limit", alt_terms_limits_[2], 1.0e6);
    getParam<double>(z_nh, "err_thresh", alt_err_thresh_, 1.0);
    getParam<double>(z_nh, "p_gain", alt_gains_[0], 0.0);
    getParam<double>(z_nh, "i_gain", alt_gains_[1], 0.001);
    getParam<double>(z_nh, "d_gain", alt_gains_[2], 0.0);
    getParam<double>(z_nh, "takeoff_i_gain", alt_takeoff_i_gain_, 0.0);

    /* xy */
    getParam<double>(xy_nh, "x_offset", xy_offset_[0], 0.0);
    getParam<double>(xy_nh, "y_offset", xy_offset_[1], 0.0);
    getParam<double>(xy_nh, "limit", xy_limit_, 1.0e6);
    getParam<double>(xy_nh, "p_term_limit", xy_terms_limits_[0], 1.0e6);
    getParam<double>(xy_nh, "i_term_limit", xy_terms_limits_[1], 1.0e6);
    getParam<double>(xy_nh, "d_term_limit", xy_terms_limits_[2], 1.0e6);
    getParam<double>(xy_nh, "p_gain", xy_gains_[0], 0.0);
    getParam<double>(xy_nh, "i_gain", xy_gains_[1], 0.0);
    getParam<double>(xy_nh, "d_gain", xy_gains_[2], 0.0);
    getParam<double>(xy_nh, "start_rp_integration_height", start_rp_integration_height_, 0.01);

    /* yaw */
    getParam<double>(yaw_nh, "err_thresh", yaw_err_thresh_, 0.4);
    getParam<double>(yaw_nh, "limit", yaw_limit_, 1.0e6);
    getParam<double>(yaw_nh, "p_term_limit", yaw_terms_limits_[0], 1.0e6);
    getParam<double>(yaw_nh, "i_term_limit", yaw_terms_limits_[1], 1.0e6);
    getParam<double>(yaw_nh, "d_term_limit", yaw_terms_limits_[2], 1.0e6);
    getParam<double>(yaw_nh, "p_gain", yaw_gains_[0], 0.0);
    getParam<double>(yaw_nh, "i_gain", yaw_gains_[1], 0.0);
    getParam<double>(yaw_nh, "d_gain", yaw_gains_[2], 0.0);

    /* roll_pitch */
    getParam<double>(roll_pitch_nh, "p_gain", roll_pitch_gains_[0], 0.0);
    getParam<double>(roll_pitch_nh, "i_gain", roll_pitch_gains_[1], 0.0);
    getParam<double>(roll_pitch_nh, "d_gain", roll_pitch_gains_[2], 0.0);
  }

  void HydrusXiFullyActuatedController::setAttitudeGains()
  {
    spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
    /* to flight controller via rosserial scaling by 1000 */
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = roll_pitch_gains_[0] * 1000;
    rpy_gain_msg.motors.at(0).roll_i = roll_pitch_gains_[1] * 1000;
    rpy_gain_msg.motors.at(0).roll_d = roll_pitch_gains_[2] * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = roll_pitch_gains_[0] * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = roll_pitch_gains_[1] * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = roll_pitch_gains_[2] * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = yaw_gains_[2] * 1000;

    rpy_gain_pub_.publish(rpy_gain_msg);
  }

  void HydrusXiFullyActuatedController::controllerGainsCfgCallback(hydrus_xi::FullyActuatedControllerGainsConfig &config, uint32_t level)
  {
    if(config.gain_flag)
      {
        ROS_INFO("FullyActuatedController Param:");
        switch(level)
          {
          case Z_P_GAIN:
            alt_gains_[0] = config.z_p;
            ROS_INFO("change the z P gain: %f\n", alt_gains_[0]);
            break;
          case Z_I_GAIN:
            alt_gains_[1] = config.z_i;
            ROS_INFO("change the z I gain: %f\n", alt_gains_[1]);
            break;
          case Z_D_GAIN:
            alt_gains_[2] = config.z_d;
            ROS_INFO("change the z D gain: %f\n", alt_gains_[2]);
            break;
          case XY_P_GAIN:
            xy_gains_[0] = config.xy_p;
            ROS_INFO("change the xy P gain: %f\n", xy_gains_[0]);
            break;
          case XY_I_GAIN:
            xy_gains_[1] = config.xy_i;
            ROS_INFO("change the xy I gain: %f\n", xy_gains_[1]);
            break;
          case XY_D_GAIN:
            xy_gains_[2] = config.xy_d;
            ROS_INFO("change the xy D gain: %f\n", xy_gains_[2]);
            break;
          case YAW_P_GAIN:
            yaw_gains_[0] = config.yaw_p;
            ROS_INFO("change the yaw P gain: %f\n", yaw_gains_[0]);
            break;
          case YAW_I_GAIN:
            yaw_gains_[1] = config.yaw_i;
            ROS_INFO("change the yaw I gain: %f\n", yaw_gains_[1]);
            break;
          case YAW_D_GAIN:
            yaw_gains_[2] = config.yaw_d;
            ROS_INFO("change the yaw D gain: %f\n", yaw_gains_[2]);
            break;
          case RP_P_GAIN:
            roll_pitch_gains_[0] = config.rp_p;
            ROS_INFO("change the roll pitch P gain: %f\n", roll_pitch_gains_[0]);
            break;
          case RP_I_GAIN:
            roll_pitch_gains_[1] = config.rp_i;
            ROS_INFO("change the roll pitch I gain: %f\n", roll_pitch_gains_[1]);
            break;
          case RP_D_GAIN:
            roll_pitch_gains_[2] = config.rp_d;
            ROS_INFO("change the roll pitch D gain: %f\n", roll_pitch_gains_[2]);
            break;
          default:
            break;
          }
        setAttitudeGains();
    }
  }

} //namespace control_plugin

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::HydrusXiFullyActuatedController, control_plugin::ControlBase);
