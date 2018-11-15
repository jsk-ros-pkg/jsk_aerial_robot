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
    RobotModelRos(ros::NodeHandle(), ros::NodeHandle("~"), std::move(std::make_unique<HydrusRobotModel>(true))),
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
    target_throttle_(0),
    target_yaw_acc_(0),
    wrench_allocation_matrix_pub_stamp_(ros::Time::now()),
    torque_allocation_matrix_inv_pub_stamp_(ros::Time::now())
  {
  }

  void HydrusXiFullyActuatedController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, Navigator* navigator, double ctrl_loop_rate) //override
  {
    ControlBase::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

    rosParamInit();

    motor_num_ = getRobotModel().getRotorNum();

    flight_cmd_pub_ = ControlBase::nh_.advertise<spinal::FourAxisCommand>("/aerial_robot_control_four_axis", 1);
    pid_pub_ = ControlBase::nh_.advertise<aerial_robot_msgs::FlatnessPid>("debug/pid", 1);

    torque_allocation_matrix_inv_pub_ = ControlBase::nh_.advertise<spinal::TorqueAllocationMatrixInv>("/torque_allocation_matrix_inv", 1);
    wrench_allocation_matrix_pub_ = ControlBase::nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
    wrench_allocation_matrix_inv_pub_ = ControlBase::nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix_inv", 1);

    set_attitude_gains_client_ = ControlBase::nh_.serviceClient<spinal::SetAttitudeGains>("/set_attitude_gains");

    dynamic_reconf_func_ = boost::bind(&HydrusXiFullyActuatedController::controllerGainsCfgCallback, this, _1, _2);
    server_.setCallback(dynamic_reconf_func_);
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
    target_throttle_.assign(motor_num_, 0);
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
    target_yaw_ = navigator_->getTargetPsi();
    target_yaw_vel_ = navigator_->getTargetPsiVel();
    yaw_err_ = target_yaw_ - state_yaw_;
    if(yaw_err_ > M_PI)  yaw_err_ -= (2 * M_PI);
  }

  void HydrusXiFullyActuatedController::pidUpdate()
  {
    aerial_robot_msgs::FlatnessPid pid_msg;
    pid_msg.header.stamp = ros::Time::now();

    /* roll/pitch integration flag */
    if(!start_rp_integration_)
      {
        if(state_pos_.z() - estimator_->getLandingHeight() > 0.01)
          {
            start_rp_integration_ = true;
            spinal::FlightConfigCmd flight_config_cmd;
            flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
            navigator_->flight_config_pub_.publish(flight_config_cmd);
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
      case flight_nav::POS_CONTROL_MODE:
        {
          /* P */
          xy_p_term = clampV(xy_gains_[0] * pos_err_, -xy_terms_limits_[0], xy_terms_limits_[0]);

          /* I */
          if (start_rp_integration_) {
            if(navigator_->getNaviState() == Navigator::TAKEOFF_STATE || navigator_->getNaviState() == Navigator::LAND_STATE) //takeoff or land
              xy_i_term_ += (pos_err_ * du * xy_gains_[1]);
            else
              xy_i_term_ += (pos_err_ * du * xy_hovering_i_gain_);
            xy_i_term_ = clampV(xy_i_term_, -xy_terms_limits_[1], xy_terms_limits_[1]);
          }

          /* D */
          vel_err_ = uav_rot_inv * (-state_vel_);
          xy_d_term = clampV(xy_gains_[2] * vel_err_, -xy_terms_limits_[2], xy_terms_limits_[2]);
          break;
        }
      case flight_nav::VEL_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */
          vel_err_ = uav_rot_inv * (target_vel_ - state_vel_);

          /* P */
          xy_p_term = clampV(xy_gains_[0] * vel_err_, -xy_terms_limits_[0], xy_terms_limits_[0]);
          xy_d_term.setZero();
          break;
        }
      case flight_nav::ACC_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */

          xy_p_term = uav_rot_inv * (target_acc_ / BasicEstimator::G);
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

    if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_err += alt_landing_const_i_ctrl_thresh_;

    /* P */
    double alt_p_term = clamp(alt_gains_[0] * alt_err, -alt_terms_limits_[0], alt_terms_limits_[0]);
    if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_p_term = 0;

    /* I */
    alt_i_term_ += alt_err * du;
    double alt_i_term = clamp(alt_gains_[1] * alt_i_term_, -alt_terms_limits_[1], alt_terms_limits_[1]);
    /* D */
    double alt_d_term = clamp(alt_gains_[2] * -state_vel_.z(), -alt_terms_limits_[2], alt_terms_limits_[2]);

    target_linear_acc_.setZ(clamp(alt_p_term + alt_i_term + alt_d_term + alt_offset_, -alt_limit_, alt_limit_));

    /* ros pub */
    //throttle(z)
    pid_msg.throttle.total.push_back(target_linear_acc_.z());
    pid_msg.throttle.p_term.push_back(alt_p_term);
    pid_msg.throttle.i_term.push_back(alt_i_term);
    pid_msg.throttle.d_term.push_back(alt_d_term);
    pid_msg.throttle.target_pos = target_pos_.z();
    pid_msg.throttle.pos_err = alt_err;
    pid_msg.throttle.target_vel = target_vel_.z();
    pid_msg.throttle.vel_err = state_vel_.z();

    /* ros publish */
    pid_pub_.publish(pid_msg);

    /* update */
    control_timestamp_ = ros::Time::now().toSec();
  }

  void HydrusXiFullyActuatedController::sendCmd()
  {
    //send flight command
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[2] = target_yaw_acc_;

    //Simple PID based position/attitude/altitude control
    const auto wrench_allocation_matrix_inv = calcWrenchAllocationMatrixInv();
    flight_command_data.base_throttle = calcForceVector(wrench_allocation_matrix_inv);

    flight_cmd_pub_.publish(flight_command_data);

    //send torque allocation matrix inv
    if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_.toSec() > torque_allocation_matrix_inv_pub_interval_)
      {
        torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now();

        spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
        torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
        Eigen::MatrixXd torque_allocation_matrix_inv = wrench_allocation_matrix_inv.block(0, 3, motor_num_, 3);
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

  Eigen::MatrixXd HydrusXiFullyActuatedController::calcWrenchAllocationMatrixInv()
  {
    std::vector<Eigen::Vector3d> rotors_origin = getRobotModel().getRotorsOriginFromCog<Eigen::Vector3d>();
    std::vector<Eigen::Vector3d> rotors_normal = getRobotModel().getRotorsNormalFromCog<Eigen::Vector3d>();
    if (rotors_origin.size() != motor_num_) {
      ROS_ERROR("HydrusXiFullyActuatedController: motor num is incorrect");
      throw "motor num is incorrect";
    }

    //Q : WrenchAllocationMatrix
    Eigen::MatrixXd Q(6, motor_num_);
    double uav_mass_inv = 1.0 / getRobotModel().getMass();
    Eigen::Matrix3d inertia_inv = getRobotModel().getInertia<Eigen::Matrix3d>().inverse();
    for (unsigned int i = 0; i < motor_num_; i++) {
      Q.block(0, i, 3, 1) = rotors_normal.at(i) * uav_mass_inv;
      Q.block(3, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)));
    }

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
    return Q_inv;
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
    ros::NodeHandle alt_node(ControlBase::nhp_, "alt");
    ros::NodeHandle xy_node(ControlBase::nhp_, "xy");
    ros::NodeHandle yaw_node(ControlBase::nhp_, "yaw");
    ros::NodeHandle roll_pitch_node(ControlBase::nhp_, "roll_pitch");

    string alt_ns = alt_node.getNamespace();
    string xy_ns = xy_node.getNamespace();
    string yaw_ns = yaw_node.getNamespace();
    string roll_pitch_ns = roll_pitch_node.getNamespace();

    ControlBase::nhp_.param("need_yaw_d_control", need_yaw_d_control_, false);
    ControlBase::nhp_.param("verbose", verbose_, false);
    ControlBase::nhp_.param("torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
    ControlBase::nhp_.param("wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);


    /* altitude */
    alt_node.param("alt_landing_const_i_ctrl_thresh", alt_landing_const_i_ctrl_thresh_, 0.0);
    alt_node.param("offset", alt_offset_, 0.0);
    alt_node.param("limit", alt_limit_, 1.0e6);
    alt_node.param("p_term_limit", alt_terms_limits_[0], 1.0e6);
    alt_node.param("i_term_limit", alt_terms_limits_[1], 1.0e6);
    alt_node.param("d_term_limit", alt_terms_limits_[2], 1.0e6);
    alt_node.param("err_thresh", alt_err_thresh_, 1.0);
    alt_node.param("p_gain", alt_gains_[0], 0.0);
    alt_node.param("i_gain", alt_gains_[1], 0.001);
    alt_node.param("d_gain", alt_gains_[2], 0.0);

    /* xy */
    xy_node.param("x_offset", xy_offset_[0], 0.0);
    xy_node.param("y_offset", xy_offset_[1], 0.0);
    xy_node.param("limit", xy_limit_, 1.0e6);
    xy_node.param("p_term_limit", xy_terms_limits_[0], 1.0e6);
    xy_node.param("i_term_limit", xy_terms_limits_[1], 1.0e6);
    xy_node.param("d_term_limit", xy_terms_limits_[2], 1.0e6);
    xy_node.param("p_gain", xy_gains_[0], 0.0);
    xy_node.param("i_gain", xy_gains_[1], 0.0);
    xy_node.param("d_gain", xy_gains_[2], 0.0);
    xy_node.param("hovering_i_gain", xy_hovering_i_gain_, 0.0);

    /* yaw */
    yaw_node.param("limit", yaw_limit_, 1.0e6);
    yaw_node.param("p_term_limit", yaw_terms_limits_[0], 1.0e6);
    yaw_node.param("i_term_limit", yaw_terms_limits_[1], 1.0e6);
    yaw_node.param("d_term_limit", yaw_terms_limits_[2], 1.0e6);
    yaw_node.param("err_thresh", yaw_err_thresh_, 0.4);
    yaw_node.param("p_gain", yaw_gains_[0], 0.0);
    yaw_node.param("i_gain", yaw_gains_[1], 0.0);
    yaw_node.param("d_gain", yaw_gains_[2], 0.0);

    /* roll_pitch */
    roll_pitch_node.param("limit", roll_pitch_limit_, 1.0e6);
    roll_pitch_node.param("p_term_limit", roll_pitch_terms_limits_[0], 1.0e6);
    roll_pitch_node.param("i_term_limit", roll_pitch_terms_limits_[1], 1.0e6);
    roll_pitch_node.param("d_term_limit", roll_pitch_terms_limits_[2], 1.0e6);
    roll_pitch_node.param("p_gain", roll_pitch_gains_[0], 0.0);
    roll_pitch_node.param("i_gain", roll_pitch_gains_[1], 0.0);
    roll_pitch_node.param("d_gain", roll_pitch_gains_[2], 0.0);


    if (param_verbose_)
      {
        cout << "verbose is " << boolalpha << verbose_ << endl;
        cout << "need_yaw_d_control_ is " << boolalpha << need_yaw_d_control_ << endl;

        cout << alt_ns << ": alt_landing_const_i_ctrl_thresh_ is " << alt_landing_const_i_ctrl_thresh_ << endl;
        cout << alt_ns << ": offset_ is " << alt_offset_ << endl;
        cout << alt_ns << ": limit_ is " << alt_limit_ << endl;
        cout << alt_ns << ": p_limit_ is " << alt_terms_limits_[0] << endl;
        cout << alt_ns << ": i_limit_ is " << alt_terms_limits_[1] << endl;
        cout << alt_ns << ": d_limit_ is " << alt_terms_limits_[2] << endl;
        cout << alt_ns << ": err_thresh_ is " << alt_err_thresh_ << endl;
        cout << alt_ns << ": p_gain_ is " << alt_gains_[0] << endl;
        cout << alt_ns << ": i_gain_ is " << alt_gains_[1] << endl;
        cout << alt_ns << ": d_gain_ is " << alt_gains_[2] << endl;

        cout << xy_ns << ": x_offset_ is " <<  xy_offset_[0] << endl;
        cout << xy_ns << ": y_offset_ is " <<  xy_offset_[1] << endl;
        cout << xy_ns << ": limit_ is " <<  xy_limit_ << endl;
        cout << xy_ns << ": p_limit_ is " <<  xy_terms_limits_[0] << endl;
        cout << xy_ns << ": i_limit_ is " <<  xy_terms_limits_[1] << endl;
        cout << xy_ns << ": d_limit_ is " <<  xy_terms_limits_[2] << endl;
        cout << xy_ns << ": p_gain_ is " << xy_gains_[0] << endl;
        cout << xy_ns << ": i_gain_ is " << xy_gains_[1] << endl;
        cout << xy_ns << ": d_gain_ is " << xy_gains_[2] << endl;
        cout << xy_ns << ": hovering_i_gain_ is " <<  xy_hovering_i_gain_ << endl;

        cout << yaw_ns << ": limit_ is " << yaw_limit_ << endl;
        cout << yaw_ns << ": p_limit_ is " << yaw_terms_limits_[0] << endl;
        cout << yaw_ns << ": i_limit_ is " << yaw_terms_limits_[1] << endl;
        cout << yaw_ns << ": d_limit_ is " << yaw_terms_limits_[2] << endl;
        cout << yaw_ns << ": err_thresh_ is " << yaw_err_thresh_ << endl;
        cout << yaw_ns << ": p_gain_ is " << yaw_gains_[0] << endl;
        cout << yaw_ns << ": i_gain_ is " << yaw_gains_[1] << endl;
        cout << yaw_ns << ": d_gain_ is " << yaw_gains_[2] << endl;

        cout << roll_pitch_ns << ": limit_ is " << roll_pitch_limit_ << endl;
        cout << roll_pitch_ns << ": p_limit_ is " << roll_pitch_terms_limits_[0] << endl;
        cout << roll_pitch_ns << ": i_limit_ is " << roll_pitch_terms_limits_[1] << endl;
        cout << roll_pitch_ns << ": d_limit_ is " << roll_pitch_terms_limits_[2] << endl;
        cout << roll_pitch_ns << ": p_gain_ is " << roll_pitch_gains_[0] << endl;
        cout << roll_pitch_ns << ": i_gain_ is " << roll_pitch_gains_[1] << endl;
        cout << roll_pitch_ns << ": d_gain_ is " << roll_pitch_gains_[2] << endl;
      }
  }

  void HydrusXiFullyActuatedController::setAttitudeGains()
  {
    spinal::SetAttitudeGains srv;
    srv.request.roll_pitch_p = roll_pitch_gains_[0];
    srv.request.roll_pitch_i = roll_pitch_gains_[1];
    srv.request.roll_pitch_d = roll_pitch_gains_[2];
    srv.request.yaw_d = yaw_gains_[2];
    srv.request.roll_pitch_limit = roll_pitch_limit_;
    srv.request.roll_pitch_p_limit = roll_pitch_terms_limits_[0];
    srv.request.roll_pitch_i_limit = roll_pitch_terms_limits_[1];
    srv.request.roll_pitch_d_limit = roll_pitch_terms_limits_[2];
    srv.request.yaw_d_limit = yaw_gains_[2];

    if (set_attitude_gains_client_.call(srv) && srv.response.success) {
      ROS_WARN("Set Attitude Gains Success");
    } else {
      ROS_ERROR("Set Attitude Gains Failure");
    }
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
    }
  }

} //namespace control_plugin
