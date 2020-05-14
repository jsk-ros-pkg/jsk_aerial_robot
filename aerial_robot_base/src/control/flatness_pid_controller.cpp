// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

/* basic class */
#include <aerial_robot_base/control/flatness_pid_controller.h>

using boost::algorithm::clamp;
using namespace std;

namespace control_plugin
{
  FlatnessPid::FlatnessPid():
    ControlBase(),
    xy_i_term_(0,0,0),
    start_rp_integration_(false), //xy (pitch/roll control) integration start
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
    target_pitch_(0),
    target_roll_(0),
    z_control_terms_(0),
    yaw_control_terms_(0),
    candidate_yaw_term_(0),
    need_yaw_d_control_(false)
  {
    xy_offset_[2] = 0;
  }

  void FlatnessPid::initialize(ros::NodeHandle nh,
                               ros::NodeHandle nhp,
                               StateEstimator* estimator,
                               Navigator* navigator,
                               double ctrl_loop_rate)
  {
    ControlBase::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

    rosParamInit();


    //publish
    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 10);
    pid_pub_ = nh_.advertise<aerial_robot_msgs::FlatnessPid>("debug/pos_yaw/pid", 10);

    //subscriber
    four_axis_gain_sub_ = nh_.subscribe<aerial_robot_msgs::FourAxisGain>("four_axes/gain", 1, &FlatnessPid::fourAxisGainCallback, this, ros::TransportHints().tcpNoDelay());

    //dynamic reconfigure server
    xy_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "gain_generator/xy"));
    dynamic_reconf_func_xy_pid_ = boost::bind(&FlatnessPid::cfgXYPidCallback, this, _1, _2);
    xy_pid_server_->setCallback(dynamic_reconf_func_xy_pid_);
  }

  bool FlatnessPid::update()
  {
    if(!ControlBase::update()) return false;

    stateError();
    pidUpdate();
    sendCmd();
  }

  void FlatnessPid::stateError()
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
    yaw_err_ = target_yaw_ - state_yaw_;
    while(yaw_err_ > M_PI)  yaw_err_ -= (2 * M_PI);
    while(yaw_err_ < -M_PI)  yaw_err_ += (2 * M_PI);
  }

  void FlatnessPid::pidUpdate()
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
    /* convert from world frame to CoG frame */
    pos_err_ = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_yaw_)) * (target_pos_ - state_pos_);
    vel_err_ = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_yaw_)) * (target_vel_ - state_vel_);

    switch(navigator_->getXyControlMode())
      {
      case flight_nav::POS_CONTROL_MODE:
        {
          /* P */
          xy_p_term = clampV(xy_gains_[0] * pos_err_,  -xy_terms_limits_[0], xy_terms_limits_[0]);

          /* I */
          if(navigator_->getNaviState() == Navigator::TAKEOFF_STATE || navigator_->getNaviState() == Navigator::LAND_STATE) //takeoff or land
            xy_i_term_ += (pos_err_ * du * xy_gains_[1]);
          else
            xy_i_term_ += (pos_err_ * du * xy_hovering_i_gain_);
          xy_i_term_ = clampV(xy_i_term_, -xy_terms_limits_[1], xy_terms_limits_[1]);

          /* D */
          xy_d_term = clampV(xy_gains_[2] * vel_err_,  -xy_terms_limits_[2], xy_terms_limits_[2]);
          break;
        }
      case flight_nav::VEL_CONTROL_MODE:
        {
          /* P */
          xy_p_term = clampV(xy_gains_[2] * vel_err_,  -xy_terms_limits_[0], xy_terms_limits_[0]);
          xy_d_term.setValue(0, 0, 0);
          break;
        }
      case flight_nav::ACC_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */
          xy_p_term = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_yaw_)) * (target_acc_ / StateEstimator::G);
          xy_i_term_.setValue(0, 0, 0);
          xy_d_term.setValue(0, 0, 0);
          break;
        }
      default:
        {
          break;
        }
      }

    tf::Vector3 xy_total_term = xy_p_term + xy_i_term_ + xy_d_term + xy_offset_;
    target_pitch_ = clamp(xy_total_term[0], -xy_limit_, xy_limit_);
    target_roll_ = clamp(-xy_total_term[1], -xy_limit_, xy_limit_); // reverse

    /* ros pub */
    pid_msg.pitch.total.push_back(target_pitch_);
    pid_msg.roll.total.push_back(target_roll_);
    pid_msg.pitch.p_term.push_back(xy_p_term[0]);
    pid_msg.pitch.i_term.push_back(xy_i_term_[0]);
    pid_msg.pitch.d_term.push_back(xy_d_term[0]);
    pid_msg.roll.p_term.push_back(xy_p_term[1]);
    pid_msg.roll.i_term.push_back(xy_i_term_[1]);
    pid_msg.roll.d_term.push_back(xy_d_term[1]);
    pid_msg.pitch.pos_err = pos_err_[0];
    pid_msg.pitch.target_pos = target_pos_[0];
    pid_msg.roll.pos_err = pos_err_[1];
    pid_msg.roll.target_pos = target_pos_[1];
    pid_msg.pitch.target_vel = target_vel_[0];
    pid_msg.roll.target_vel = target_vel_[1];
    pid_msg.pitch.vel_err = vel_err_[0];
    pid_msg.roll.vel_err = vel_err_[1];

    /* yaw */
    std::vector<double> yaw_control_terms_tmp(yaw_control_terms_);
    max_yaw_term_ = 0;
    int16_t max_yaw_d_gain = 0; // for reconstruct yaw control term in spinal

    double yaw_err = clamp(yaw_err_, -yaw_err_thresh_, yaw_err_thresh_);
    yaw_err_i_ += yaw_err * du;
    for(int j = 0; j < motor_num_; j++)
      {
        //**** P term
        double yaw_p_term = clamp(-yaw_gains_[j][0] * yaw_err, -yaw_terms_limits_[0], yaw_terms_limits_[0]);

        //**** I term:
        double yaw_i_term = clamp(yaw_gains_[j][1] * yaw_err_i_, -yaw_terms_limits_[1], yaw_terms_limits_[1]);

        //***** D term: usaully it is in the flight board
        /* but for the gimbal control, we need the d term, set 0 if it is not gimbal type */
        double yaw_d_term = -yaw_gains_[j][2] * target_yaw_vel_;
        if(need_yaw_d_control_) yaw_d_term += (-yaw_gains_[j][2] * (-state_yaw_vel_));
        yaw_d_term = clamp(yaw_d_term, -yaw_terms_limits_[2], yaw_terms_limits_[2]);

        //*** each motor command value for log
        yaw_control_terms_tmp[j] = yaw_p_term + yaw_i_term + yaw_d_term;

        pid_msg.yaw.total.push_back(yaw_control_terms_tmp[j]);
        pid_msg.yaw.p_term.push_back(yaw_p_term);
        pid_msg.yaw.i_term.push_back(yaw_i_term);
        pid_msg.yaw.d_term.push_back(yaw_d_term);

        if(yaw_gains_.size() == 1) break;

        if(fabs(yaw_control_terms_tmp[j]) > max_yaw_term_) max_yaw_term_ = fabs(yaw_control_terms_tmp[j]);

        /* use d gains to find the maximum (positive) value */
        /* only select positve terms to avoid identical absolute value */
        if(static_cast<int16_t>(yaw_gains_[j][2] * 1000) > max_yaw_d_gain)
          {
            max_yaw_d_gain = static_cast<int16_t>(yaw_gains_[j][2] * 1000);
            candidate_yaw_term_ = yaw_control_terms_tmp[j];
          }
      }

    if(max_yaw_term_ <= yaw_limit_)
      std::copy(yaw_control_terms_tmp.begin(), yaw_control_terms_tmp.end(), yaw_control_terms_.begin());
    else
      yaw_err_i_ -= yaw_err * du; // do not increase this term if saturated

    //**** ros pub
    pid_msg.yaw.target_pos = target_yaw_;
    pid_msg.yaw.pos_err = yaw_err_;
    pid_msg.yaw.target_vel = target_yaw_vel_;

    /* z */
    std::vector<double> z_control_terms_tmp(z_control_terms_);
    double max_z_control_terms = 0;

    double z_pos_err = clamp(pos_err_.z(), -z_err_thresh_, z_err_thresh_);
    if(navigator_->getNaviState() == Navigator::LAND_STATE && -pos_err_.z() > safe_landing_height_)
      {
        /* too high, slowly descend */
        z_pos_err = landing_z_err_thresh_;

        /* avoid the unexceped ascending when the i term exceed the hovering state */
        if(state_vel_.z() > landing_z_err_thresh_) z_pos_err_i_ += z_pos_err * du;
      }
    else
      {
        z_pos_err_i_ += z_pos_err * du;

        if(navigator_->getNaviState() == Navigator::LAND_STATE)
          z_pos_err = 0; // no p control in final safe landing phase
      }
    double z_vel_err = target_vel_.z() - state_vel_.z();

    for(int j = 0; j < motor_num_; j++)
      {
        //**** P Term
        double z_p_term = clamp(-z_gains_[j][0] * z_pos_err, -z_terms_limit_[0], z_terms_limit_[0]);

        //**** I Term
        double z_i_term = clamp(z_gains_[j][1] * z_pos_err_i_, -z_terms_limit_[1], z_terms_limit_[1]);
        //***** D Term
        double z_d_term = clamp(-z_gains_[j][2] * z_vel_err, -z_terms_limit_[2], z_terms_limit_[2]);

        z_control_terms_tmp[j] = z_p_term + z_i_term + z_d_term + z_offset_;

        pid_msg.z.total.push_back(z_control_terms_tmp[j]);
        pid_msg.z.p_term.push_back(z_p_term);
        pid_msg.z.i_term.push_back(z_i_term);
        pid_msg.z.d_term.push_back(z_d_term);

        if(z_gains_.size() == 1)
          {
            z_control_terms_[j] = clamp(z_control_terms_tmp[j], 0, z_limit_);
            break;
          }

        if(z_control_terms_tmp[j] > max_z_control_terms)
          max_z_control_terms = z_control_terms_tmp[j];
      }

    if(max_z_control_terms <= z_limit_)
      std::copy(z_control_terms_tmp.begin(), z_control_terms_tmp.end(), z_control_terms_.begin());
    else
      z_pos_err_i_ -=  z_pos_err * du; // do not increase this term if saturated


    pid_msg.z.target_pos = target_pos_.z();
    pid_msg.z.pos_err = z_pos_err;
    pid_msg.z.target_vel = target_vel_.z();
    pid_msg.z.vel_err = z_vel_err;

    /* ros publish */
    pid_pub_.publish(pid_msg);

    /* update */
    control_timestamp_ = ros::Time::now().toSec();
  }

  void FlatnessPid::sendCmd()
  {
    /* send flight command */
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] =  target_roll_;
    flight_command_data.angles[1] =  target_pitch_;
    flight_command_data.angles[2] = candidate_yaw_term_;

    flight_command_data.base_throttle.resize(motor_num_);

    if(uav_model_ == spinal::UavInfo::DRONE)
      {
        /* Simple PID based attitude/zitude control */
        flight_command_data.base_throttle[0] =  z_control_terms_[0];
        if(z_control_terms_[0] == 0) return; // do not publish the empty flight command => the force landing flag will be activate
      }
    else
      {
        /* LQI based attitude/zitude control */
        for(int i = 0; i < motor_num_; i++)
          flight_command_data.base_throttle[i] = z_control_terms_[i];
      }
    flight_cmd_pub_.publish(flight_command_data);
  }

  void FlatnessPid::fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
  {
    /* update the motor number */
    if(motor_num_ == 0)
      {
        motor_num_ = msg->motor_num;

        yaw_gains_.resize(motor_num_);
        z_gains_.resize(motor_num_);

        z_control_terms_.resize(motor_num_);
        yaw_control_terms_.resize(motor_num_);

        ROS_INFO("Flight control: update the motor number from gain message: %d", motor_num_);
      }

    for(int i = 0; i < msg->motor_num; i++)
      {
        yaw_gains_[i].setValue(msg->pos_p_gain_yaw[i], msg->pos_i_gain_yaw[i], msg->pos_d_gain_yaw[i]);
        z_gains_[i].setValue(msg->pos_p_gain_z[i], msg->pos_i_gain_z[i], msg->pos_d_gain_z[i]);
      }
  }


  void FlatnessPid::cfgXYPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level)
  {
    if(config.xy_pid_control_flag)
      {
        printf("XY Pid Param:");
        switch(level)
          {
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
            xy_gains_[0] = config.p_gain;
            printf("change the p gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
            xy_gains_[1] = config.i_gain;
            printf("change the i gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
            xy_gains_[2] = config.d_gain;
            printf("change the d gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN_HOVER:
            xy_hovering_i_gain_ = config.i_gain_hover;
            printf("change the i hovering gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
            xy_limit_ = config.limit;
            printf("change the limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
            xy_terms_limits_[0] = config.p_limit;
            printf("change the p limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
            xy_terms_limits_[1] = config.i_limit;
            printf("change the i limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
            xy_terms_limits_[2] = config.d_limit;
            printf("change the d limit\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }

  void FlatnessPid::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle xy_nh(control_nh, "xy");
    ros::NodeHandle z_nh(control_nh, "z");
    ros::NodeHandle yaw_nh(control_nh, "yaw");

    /* z */
    getParam<double>(z_nh, "landing_z_err_thresh", landing_z_err_thresh_, -0.5);
    if(landing_z_err_thresh_ >=0) landing_z_err_thresh_  = -0.5;
    getParam<double>(z_nh, "safe_landing_height",  safe_landing_height_, 0.5);
    getParam<double>(z_nh, "offset", z_offset_, 0.0);
    getParam<double>(z_nh, "limit", z_limit_, 1.0e6);
    getParam<double>(z_nh, "p_term_limit", z_terms_limit_[0], 1.0e6);
    getParam<double>(z_nh, "i_term_limit", z_terms_limit_[1], 1.0e6);
    getParam<double>(z_nh, "d_term_limit", z_terms_limit_[2], 1.0e6);
    getParam<double>(z_nh, "err_thresh", z_err_thresh_, 1.0);
    z_gains_.resize(1);
    getParam<double>(z_nh, "p_gain", z_gains_[0][0], 0.0);
    getParam<double>(z_nh, "i_gain", z_gains_[0][1], 0.001);
    getParam<double>(z_nh, "d_gain", z_gains_[0][2], 0.0);

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
    getParam<double>(xy_nh, "hovering_i_gain", xy_hovering_i_gain_, 0.0);

    /* yaw */
    getParam<double>(yaw_nh, "limit", yaw_limit_, 1.0e6);
    getParam<double>(yaw_nh, "p_term_limit", yaw_terms_limits_[0], 1.0e6);
    getParam<double>(yaw_nh, "i_term_limit", yaw_terms_limits_[1], 1.0e6);
    getParam<double>(yaw_nh, "d_term_limit", yaw_terms_limits_[2], 1.0e6);
    yaw_gains_.resize(1); /* default is for general multirotor */
    getParam<double>(yaw_nh, "p_gain", yaw_gains_[0][0], 0.0);
    getParam<double>(yaw_nh, "i_gain", yaw_gains_[0][1], 0.0);
    getParam<double>(yaw_nh, "d_gain", yaw_gains_[0][2], 0.0);
    getParam<double>(yaw_nh, "err_thresh", yaw_err_thresh_, 0.4);
  }
};
