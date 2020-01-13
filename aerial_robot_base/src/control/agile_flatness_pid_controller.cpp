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

/* basic class */
#include <aerial_robot_base/control/agile_flatness_pid_controller.h>

namespace control_plugin
{
  AgileFlatnessPid::AgileFlatnessPid():
    FlatnessPid()
  {
  }
  void AgileFlatnessPid::initialize(ros::NodeHandle nh,
                               ros::NodeHandle nhp,
                               StateEstimator* estimator,
                               Navigator* navigator,
                               double ctrl_loop_rate)
  {
    FlatnessPid::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);
  }

  void AgileFlatnessPid::pidUpdate()
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
    pos_err_ = target_pos_ - state_pos_;
    vel_err_ = target_vel_ - state_vel_;

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
          xy_p_term = target_acc_;
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
    xy_total_term[0] = clamp(xy_total_term[0], -xy_limit_, xy_limit_);
    xy_total_term[1] = clamp(xy_total_term[1], -xy_limit_, xy_limit_);

    /* ros pub */
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

    /* z */
    std::vector<double> target_throttle_tmp(target_throttle_);
    double max_target_throttle = 0;

    double alt_pos_err = clamp(pos_err_.z(), -alt_err_thresh_, alt_err_thresh_);
    alt_pos_err_i_ +=  alt_pos_err * du;
    double alt_vel_err = target_vel_.z() - state_vel_.z();

    if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_pos_err += alt_landing_const_i_ctrl_thresh_;

    for(int j = 0; j < motor_num_; j++)
      {
        //**** P Term
        double alt_p_term = clamp(-alt_gains_[j][0] * alt_pos_err, -alt_terms_limit_[0], alt_terms_limit_[0]);
        if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_p_term = 0;

        //**** I Term
        double alt_i_term = clamp(alt_gains_[j][1] * alt_pos_err_i_, -alt_terms_limit_[1], alt_terms_limit_[1]);
        //***** D Term
        double alt_d_term = clamp(-alt_gains_[j][2] * alt_vel_err, -alt_terms_limit_[2], alt_terms_limit_[2]);

        target_throttle_tmp[j] = alt_p_term + alt_i_term + alt_d_term + alt_offset_;

        pid_msg.throttle.p_term.push_back(alt_p_term);
        pid_msg.throttle.i_term.push_back(alt_i_term);
        pid_msg.throttle.d_term.push_back(alt_d_term);

        if(alt_gains_.size() == 1)
          {
            target_throttle_[j] = clamp(target_throttle_tmp[j], 0, alt_limit_);
            break;
          }

        if(target_throttle_tmp[j] > max_target_throttle)
          max_target_throttle = target_throttle_tmp[j];
      }

    if(max_target_throttle <= alt_limit_)
      std::copy(target_throttle_tmp.begin(), target_throttle_tmp.end(), target_throttle_.begin());
    else
      alt_pos_err_i_ -=  alt_pos_err * du; // do not increase this term if saturated

    double z_total_term = std::accumulate(target_throttle_.begin(), target_throttle_.end(), 0.0);

    pid_msg.throttle.target_pos = target_pos_.z();
    pid_msg.throttle.pos_err = alt_pos_err;
    pid_msg.throttle.target_vel = target_vel_.z();
    pid_msg.throttle.vel_err = alt_vel_err;

    /* change from desired accelaration (force) to desired roll/pitch and throttle */
    z_total_term /= estimator_->getMass();
    tf::Vector3 desired_force(xy_total_term[0], xy_total_term[1], z_total_term);

    double desired_total_throttle = desired_force.length();
    tf::Vector3 desired_force_cog_frame = (tf::Matrix3x3(tf::createQuaternionFromYaw(state_psi_))).inverse() * desired_force;

    for(int j = 0; j < motor_num_; j++)
      {
        target_throttle_[j] *= desired_total_throttle / z_total_term;
        pid_msg.throttle.total.push_back(target_throttle_[j]);
        if(alt_gains_.size() == 1) break;
      }

    target_pitch_ = atan2(desired_force_cog_frame.x(), desired_force_cog_frame.z());
    target_roll_ = atan2(-desired_force_cog_frame.y(), sqrt(desired_force_cog_frame.x() * desired_force_cog_frame.x() + desired_force_cog_frame.z() * desired_force_cog_frame.z()));

    pid_msg.pitch.total.push_back(target_pitch_);
    pid_msg.roll.total.push_back(target_roll_);

    /* yaw */
    std::vector<double> target_yaw_tmp(target_yaw_);
    max_target_yaw_ = 0;
    int16_t max_yaw_d_gain = 0; // for reconstruct yaw control term in spinal

    double psi_err = clamp(psi_err_, -yaw_err_thresh_, yaw_err_thresh_);
    psi_err_i_ += psi_err * du;
    for(int j = 0; j < motor_num_; j++)
      {
        //**** P term
        double yaw_p_term = clamp(-yaw_gains_[j][0] * psi_err, -yaw_terms_limits_[0], yaw_terms_limits_[0]);

        //**** I term:
        double yaw_i_term = clamp(yaw_gains_[j][1] * psi_err_i_, -yaw_terms_limits_[1], yaw_terms_limits_[1]);

        //***** D term: usaully it is in the flight board
        /* but for the gimbal control, we need the d term, set 0 if it is not gimbal type */
        double yaw_d_term = -yaw_gains_[j][2] * target_psi_vel_;
        if(need_yaw_d_control_) yaw_d_term += (-yaw_gains_[j][2] * (-state_psi_vel_));
        yaw_d_term = clamp(yaw_d_term, -yaw_terms_limits_[2], yaw_terms_limits_[2]);

        //*** each motor command value for log
        target_yaw_tmp[j] = yaw_p_term + yaw_i_term + yaw_d_term;

        pid_msg.yaw.total.push_back(target_yaw_tmp[j]);
        pid_msg.yaw.p_term.push_back(yaw_p_term);
        pid_msg.yaw.i_term.push_back(yaw_i_term);
        pid_msg.yaw.d_term.push_back(yaw_d_term);

        if(yaw_gains_.size() == 1) break;

        if(fabs(target_yaw_tmp[j]) > max_target_yaw_) max_target_yaw_ = fabs(target_yaw_tmp[j]);

        /* use d gains to find the maximum (positive) value */
        /* only select positve terms to avoid identical absolute value */
        if(static_cast<int16_t>(yaw_gains_[j][2] * 1000) > max_yaw_d_gain)
          {
            max_yaw_d_gain = static_cast<int16_t>(yaw_gains_[j][2] * 1000);
            candidate_yaw_term_ = target_yaw_tmp[j];
          }
      }

    if(max_target_yaw_ <= yaw_limit_)
      std::copy(target_yaw_tmp.begin(), target_yaw_tmp.end(), target_yaw_.begin());
    else
      psi_err_i_ -= psi_err * du; // do not increase this term if saturated

    //**** ros pub
    pid_msg.yaw.target_pos = target_psi_;
    pid_msg.yaw.pos_err = psi_err_;
    pid_msg.yaw.target_vel = target_psi_vel_;

    /* ros publish */
    pid_pub_.publish(pid_msg);

    /* update */
    control_timestamp_ = ros::Time::now().toSec();
  }
};
