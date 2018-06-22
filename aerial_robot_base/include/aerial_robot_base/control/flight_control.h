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

#ifndef FLIGHT_CONTROLLER_BASE_PLUGIN_H
#define FLIGHT_CONTROLLER_BASE_PLUGIN_H

/* ros */
#include <ros/ros.h>

/* basic instance */
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/flight_navigation.h>

/* ros msg to Spinal */
#include <spinal/PwmInfo.h>
#include <spinal/UavInfo.h>
#include <spinal/FourAxisCommand.h>

/* util */
#include <boost/algorithm/clamp.hpp>
#include <tf/transform_datatypes.h>

using namespace std;

namespace control_plugin
{
  class ControlBase
  {
  public:
    ControlBase(): control_timestamp_(-1)
    {}

    virtual ~ControlBase(){}
    void virtual initialize(ros::NodeHandle nh,
               ros::NodeHandle nhp,
               BasicEstimator* estimator,
               Navigator* navigator,
               double ctrl_loop_rate)
    {
      nh_ = ros::NodeHandle(nh, "controller");
      nhp_ = ros::NodeHandle(nhp,  "controller");

      motor_info_pub_ = nh_.advertise<spinal::PwmInfo>("/motor_info", 10);
      uav_info_pub_ = nh_.advertise<spinal::UavInfo>("/uav_info", 10);

      estimator_ = estimator;
      navigator_ = navigator;

      ctrl_loop_rate_ = ctrl_loop_rate;

      estimate_mode_ = estimator_->getEstimateMode();

      nhp.param("param_verbose", param_verbose_, false);

      ros::NodeHandle motor_info_node("motor_info");
      std::string ns = motor_info_node.getNamespace();

      motor_info_node.param("min_thrust", min_thrust_, 0.0);
      if(param_verbose_) cout << ns  << ": min_thrust_ is "  <<  min_thrust_ << endl;
      motor_info_node.param("max_thrust", max_thrust_, 0.0);
      if(param_verbose_) cout << ns  << ": max_thrust_ is "  <<  max_thrust_ << endl;
      motor_info_node.param("abs_max_pwm", abs_max_pwm_, 0.0);
      if(param_verbose_) cout << ns  << ": abs_max_pwm_ is "  <<  abs_max_pwm_ << endl;
      motor_info_node.param("force_landing_thrust", force_landing_thrust_, 0.0);
      if(param_verbose_) cout << ns  << ": force_landing_thrust_ is "  <<  force_landing_thrust_ << endl;
      motor_info_node.param("m_f_rate", m_f_rate_, 0.0);
      if(param_verbose_) cout << ns  << ": m_f_rate_ is "  <<  m_f_rate_ << endl;
      motor_info_node.param("pwm_conversion_mode", pwm_conversion_mode_, -1);
      if(param_verbose_) cout << ns  << ": pwm_conversion_mode_ is "  <<  pwm_conversion_mode_ << endl;
      int vel_ref_num;
      motor_info_node.param("vel_ref_num", vel_ref_num, 0);
      if(param_verbose_) cout << ns  << ": vel_ref_num is "  <<  vel_ref_num << endl;
      motor_info_.resize(vel_ref_num);
      for(int i = 0; i < vel_ref_num; i++)
        {
          std::stringstream ss;
          ss << i + 1;
          double val;
          ros::NodeHandle nh(motor_info_node, "ref" + ss.str());
          nh.param("voltage", val, 0.0);
          motor_info_[i].voltage = val;
          if(param_verbose_) cout << nh.getNamespace()  << ": voltage is "  <<  val << endl;
          /* hardcode: up to 4 dimension */
          for(int j = 0; j < 5; j++)
            {
              std::stringstream ss2;
              ss2 << j;
              nh.param("polynominal" + ss2.str(), val, 0.0);
              motor_info_[i].polynominal[j] = val;
              if(param_verbose_) cout << nh.getNamespace() << ": polynominal" << j << " is "  <<  val << endl;
            }
        }

      ros::NodeHandle uav_info_node("uav_info");
      ns = uav_info_node.getNamespace();
      uav_info_node.param("uav_model", uav_model_, 0); //0: DRONE
      if(param_verbose_) cout << ns  << ": uav_model_ is "  <<  uav_model_ << endl;
      /* the motor number can be calculated from ros model(KDL), so this is not necessary */
      uav_info_node.param("motor_num", motor_num_, 0);
      if(param_verbose_) cout << ns  << ": motor_num_ is "  <<  motor_num_ << endl;
    }

    virtual bool update()
    {
      if(motor_num_ == 0) return false;

      if(navigator_->getNaviState() == Navigator::START_STATE) activate();
      if(navigator_->getNaviState() == Navigator::ARM_OFF_STATE && control_timestamp_ > 0)
        {
          reset();
        }

      if (control_timestamp_ < 0)
        {
          if (navigator_->getNaviState() == Navigator::TAKEOFF_STATE)
            {
              reset();
              control_timestamp_ = ros::Time::now().toSec();
            }
          else return false;
        }

      return true;
    }

    virtual void activate()
    {
      /* motor related info */
      /* initialize setting */

      static ros::Time activate_timestamp = ros::Time(0);
      if(ros::Time::now().toSec() - activate_timestamp.toSec()  > 0.1)
        {
          /* send motor and uav info to uav, about 10Hz */
          spinal::PwmInfo motor_info_msg;
          motor_info_msg.min_thrust = min_thrust_;
          motor_info_msg.max_thrust = max_thrust_;
          motor_info_msg.abs_max_pwm = abs_max_pwm_;
          motor_info_msg.force_landing_thrust = force_landing_thrust_;
          motor_info_msg.pwm_conversion_mode = pwm_conversion_mode_;
          motor_info_msg.motor_info.resize(0);
          for(int i = 0; i < motor_info_.size(); i++)
            motor_info_msg.motor_info.push_back(motor_info_[i]);
          motor_info_pub_.publish(motor_info_msg);

          spinal::UavInfo uav_info_msg;
          uav_info_msg.motor_num = motor_num_;
          uav_info_msg.uav_model = uav_model_;
          uav_info_pub_.publish(uav_info_msg);

          activate_timestamp = ros::Time::now();

        }
      reset();
    }

    virtual void reset()
    {
      control_timestamp_ = -1;
    }

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher  motor_info_pub_;
    ros::Publisher  uav_info_pub_;

    Navigator* navigator_;
    BasicEstimator* estimator_;

    double ctrl_loop_rate_;
    double control_timestamp_;
    int motor_num_;
    int uav_model_;

    /* new param */
    double min_thrust_, max_thrust_;
    double abs_max_pwm_;
    double m_f_rate_;
    double force_landing_thrust_; //pwm
    int pwm_conversion_mode_;
    std::vector<spinal::MotorInfo> motor_info_;

    int estimate_mode_;

    bool param_verbose_;
  };

};
#endif
