// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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


#include <aerial_robot_control/control/pose_linear_controller.h>

namespace aerial_robot_control
{
  PoseLinearController::PoseLinearController():
    ControlBase(),
    pid_controllers_(0),
    pid_reconf_servers_(0),
    pos_(0,0,0), target_pos_(0,0,0),
    vel_(0,0,0), target_vel_(0,0,0),
    rpy_(0,0,0), target_rpy_(0,0,0),
    target_acc_(0,0,0),
    start_rp_integration_(false)
  {
    pid_msg_.x.total.resize(1);
    pid_msg_.x.p_term.resize(1);
    pid_msg_.x.i_term.resize(1);
    pid_msg_.x.d_term.resize(1);
    pid_msg_.y.total.resize(1);
    pid_msg_.y.p_term.resize(1);
    pid_msg_.y.i_term.resize(1);
    pid_msg_.y.d_term.resize(1);
    pid_msg_.z.total.resize(1);
    pid_msg_.z.p_term.resize(1);
    pid_msg_.z.i_term.resize(1);
    pid_msg_.z.d_term.resize(1);
    pid_msg_.roll.total.resize(1);
    pid_msg_.roll.p_term.resize(1);
    pid_msg_.roll.i_term.resize(1);
    pid_msg_.roll.d_term.resize(1);
    pid_msg_.pitch.total.resize(1);
    pid_msg_.pitch.p_term.resize(1);
    pid_msg_.pitch.i_term.resize(1);
    pid_msg_.pitch.d_term.resize(1);
    pid_msg_.yaw.total.resize(1);
    pid_msg_.yaw.p_term.resize(1);
    pid_msg_.yaw.i_term.resize(1);
    pid_msg_.yaw.d_term.resize(1);
  }

  void PoseLinearController::initialize(ros::NodeHandle nh,
                               ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                               boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                               double ctrl_loop_rate)
  {
    ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    ros::NodeHandle control_nh(nh_, "controller");

    ros::NodeHandle xy_nh(control_nh, "xy");
    ros::NodeHandle x_nh(control_nh, "x");
    ros::NodeHandle y_nh(control_nh, "y");
    ros::NodeHandle z_nh(control_nh, "z");

    ros::NodeHandle roll_pitch_nh(control_nh, "roll_pitch");
    ros::NodeHandle roll_nh(control_nh, "roll");
    ros::NodeHandle pitch_nh(control_nh, "pitch");
    ros::NodeHandle yaw_nh(control_nh, "yaw");

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

    /* xy */
    if(xy_nh.hasParam("p_gain"))
      {
        loadParam(xy_nh);
        pid_controllers_.push_back(PID("x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        pid_controllers_.push_back(PID("y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

        std::vector<int> indices = {X, Y};
        pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(xy_nh));
        pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, indices));
      }
    else
      {
        loadParam(x_nh);
        pid_controllers_.push_back(PID("x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(x_nh));
        pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, std::vector<int>(1, X)));

        loadParam(y_nh);
        pid_controllers_.push_back(PID("y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(y_nh));
        pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, std::vector<int>(1, Y)));
      }

    /* z */
    getParam<double>(z_nh, "landing_err_z", landing_err_z_, -0.5);
    getParam<double>(z_nh, "safe_landing_height",  safe_landing_height_, 0.5);

    loadParam(z_nh);
    pid_controllers_.push_back(PID("z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
    pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(z_nh));
    pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, std::vector<int>(1, Z)));

    /* roll pitch */
    getParam<double>(roll_pitch_nh, "start_integration_height", start_rp_integration_height_, 0.01);
    if(roll_pitch_nh.hasParam("p_gain"))
      {
        loadParam(roll_pitch_nh);
        pid_controllers_.push_back(PID("roll", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        pid_controllers_.push_back(PID("pitch", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        std::vector<int> indices = {ROLL, PITCH};
        pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(roll_pitch_nh));
        pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, indices));
      }
    else
      {
        loadParam(roll_nh);
        pid_controllers_.push_back(PID("roll", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(roll_nh));
        pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, std::vector<int>(1, ROLL)));

        loadParam(pitch_nh);
        pid_controllers_.push_back(PID("pitch", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
        pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(pitch_nh));
        pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, std::vector<int>(1, PITCH)));
      }

    /* yaw */
    loadParam(yaw_nh);
    getParam<bool>(yaw_nh, "need_d_control", need_yaw_d_control_, false);
    pid_controllers_.push_back(PID("yaw", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
    pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(yaw_nh));
    pid_reconf_servers_.back()->setCallback(boost::bind(&PoseLinearController::cfgPidCallback, this, _1, _2, std::vector<int>(1, YAW)));


    pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/pose/pid", 10);
    yaw_from_pc_pub_ = nh_.advertise<spinal::YawFromPC>("yaw_from_pc", 1);
  }

  void PoseLinearController::reset()
  {
    ControlBase::reset();
    start_rp_integration_ = false;

    for(auto& controller: pid_controllers_) controller.reset();
  }

  bool PoseLinearController::update()
  {
    if(!ControlBase::update()) return false;

    controlCore();
    sendCmd();
  }

  void PoseLinearController::controlCore()
  {
    pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
    vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
    target_pos_ = navigator_->getTargetPos();
    target_vel_ = navigator_->getTargetVel();
    target_acc_ = navigator_->getTargetAcc();

    rpy_ = estimator_->getEuler(Frame::COG, estimate_mode_);
    omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);
    target_rpy_ = navigator_->getTargetRPY();
    target_omega_ = navigator_->getTargetOmega();

    // time diff
    double du = ros::Time::now().toSec() - control_timestamp_;

    // roll/pitch integration flag
    if(!start_rp_integration_)
      {
        if(pos_.z() - estimator_->getLandingHeight() > start_rp_integration_height_)
          {
            start_rp_integration_ = true;
            spinal::FlightConfigCmd flight_config_cmd;
            flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
            navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
            ROS_WARN("start roll/pitch I control");
          }
      }


    // x & y
    switch(navigator_->getXyControlMode())
      {
      case aerial_robot_navigation::POS_CONTROL_MODE:
        pid_controllers_.at(X).update(target_pos_.x() - pos_.x(), du, target_vel_.x() - vel_.x(), target_acc_.x());
        pid_controllers_.at(Y).update(target_pos_.y() - pos_.y(), du, target_vel_.y() - vel_.y(), target_acc_.y());
        break;
      case aerial_robot_navigation::VEL_CONTROL_MODE:
        pid_controllers_.at(X).update(0, du, target_vel_.x() - vel_.x(), target_acc_.x());
        pid_controllers_.at(Y).update(0, du, target_vel_.y() - vel_.y(), target_acc_.y());
        break;
      case aerial_robot_navigation::ACC_CONTROL_MODE:
        pid_controllers_.at(X).update(0, du, 0, target_acc_.x());
        pid_controllers_.at(Y).update(0, du, 0, target_acc_.y());
        break;
      default:
        break;
      }

    // z
    double err_z = target_pos_.z() - pos_.z();
    double du_z = du;
    double z_p_limit = pid_controllers_.at(Z).getLimitP();
    bool final_landing_phase = false;
    if(navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE)
      {
        if(-err_z > safe_landing_height_)
          {
            err_z = landing_err_z_;  // too high, slowly descend
            if(vel_.z() < landing_err_z_) du_z = 0;  // freeze i term when descending
          }
        else
          {
            pid_controllers_.at(Z).setLimitP(0); // no p control in final safe landing phase
            final_landing_phase = true;
          }
      }
    pid_controllers_.at(Z).update(err_z, du_z, target_vel_.z() - vel_.z(), target_acc_.z());

    if(final_landing_phase)
      {
        pid_controllers_.at(Z).setLimitP(z_p_limit); // revert z p limit
        pid_controllers_.at(Z).setErrP(0); // for derived controller which use err_p in feedback control (e.g., LQI)
      }

    // roll pitch
    pid_controllers_.at(ROLL).update(target_rpy_.x() - rpy_.x(), du, target_omega_.x() - omega_.x());
    pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du, target_omega_.y() - omega_.y());

    // yaw
    double err_yaw = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
    double err_omega_z = target_omega_.z() - omega_.z();
    if(!need_yaw_d_control_)
      {
        err_omega_z = target_omega_.z(); // part of the control in spinal
      }
    pid_controllers_.at(YAW).update(err_yaw, du, err_omega_z);

    // update
    control_timestamp_ = ros::Time::now().toSec();

    /* ros pub */
    pid_msg_.header.stamp = ros::Time::now();
    pid_msg_.x.total.at(0) = pid_controllers_.at(X).result();
    pid_msg_.x.p_term.at(0) = pid_controllers_.at(X).getPTerm();
    pid_msg_.x.i_term.at(0) = pid_controllers_.at(X).getITerm();
    pid_msg_.x.d_term.at(0) = pid_controllers_.at(X).getDTerm();
    pid_msg_.x.target_p = target_pos_.x();
    pid_msg_.x.err_p = target_pos_.x() - pos_.x();
    pid_msg_.x.target_d = target_vel_.x();
    pid_msg_.x.err_d = target_vel_.x() - vel_.x();

    pid_msg_.y.total.at(0) = pid_controllers_.at(Y).result();
    pid_msg_.y.p_term.at(0) = pid_controllers_.at(Y).getPTerm();
    pid_msg_.y.i_term.at(0) = pid_controllers_.at(Y).getITerm();
    pid_msg_.y.d_term.at(0) = pid_controllers_.at(Y).getDTerm();
    pid_msg_.y.target_p = target_pos_.y();
    pid_msg_.y.err_p = target_pos_.y() - pos_.y();
    pid_msg_.y.target_d = target_vel_.y();
    pid_msg_.y.err_d = target_vel_.y() - vel_.y();

    pid_msg_.z.total.at(0) = pid_controllers_.at(Z).result();
    pid_msg_.z.p_term.at(0) = pid_controllers_.at(Z).getPTerm();
    pid_msg_.z.i_term.at(0) = pid_controllers_.at(Z).getITerm();
    pid_msg_.z.d_term.at(0) = pid_controllers_.at(Z).getDTerm();
    pid_msg_.z.target_p = target_pos_.z();
    pid_msg_.z.err_p = target_pos_.z() - pos_.z();
    pid_msg_.z.target_d = target_vel_.z();
    pid_msg_.z.err_d = target_vel_.z() - vel_.z();

    // omit roll, pitch here

    pid_msg_.yaw.total.at(0) = pid_controllers_.at(YAW).result();
    pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(YAW).getPTerm();
    pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(YAW).getITerm();
    pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(YAW).getDTerm();
    pid_msg_.yaw.target_p = target_rpy_.z();
    pid_msg_.yaw.err_p = err_yaw;
    pid_msg_.yaw.target_d = target_omega_.z();
    pid_msg_.yaw.err_d = target_omega_.z() - omega_.z();

    yaw_from_pc_msg_.pc_yaw = rpy_.z();
    yaw_from_pc_msg_.target_yaw = target_rpy_.z();
  }

  void PoseLinearController::sendCmd()
  {
    /* ros publish */
    pid_pub_.publish(pid_msg_);
    yaw_from_pc_pub_.publish(yaw_from_pc_msg_);
  }

  void PoseLinearController::cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices)
  {
    using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
    if(config.pid_control_flag)
      {
        switch(level)
          {
          case Levels::RECONFIGURE_P_GAIN:
            for(const auto& index: controller_indices)
              {
                pid_controllers_.at(index).setPGain(config.p_gain);
                ROS_INFO_STREAM("change p gain for controller '" << pid_controllers_.at(index).getName() << "'");
              }
            break;
          case Levels::RECONFIGURE_I_GAIN:
            for(const auto& index: controller_indices)
              {
                pid_controllers_.at(index).setIGain(config.i_gain);
                ROS_INFO_STREAM("change i gain for controller '" << pid_controllers_.at(index).getName() << "'");
              }
            break;
          case Levels::RECONFIGURE_D_GAIN:
            for(const auto& index: controller_indices)
              {
                pid_controllers_.at(index).setDGain(config.d_gain);
                ROS_INFO_STREAM("change d gain for controller '" << pid_controllers_.at(index).getName() << "'");
              }
            break;
          default :
            break;
          }
      }
  }
};
