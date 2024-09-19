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

#pragma once


#include <aerial_robot_control/flight_navigation.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <ros/ros.h>
#include <spinal/PwmInfo.h>
#include <spinal/UavInfo.h>

namespace aerial_robot_control
{
  class ControlBase
  {
  public:
    ControlBase(): control_timestamp_(-1), activate_timestamp_(0)
    {}

    virtual ~ControlBase(){}
    void virtual initialize(ros::NodeHandle nh,
                            ros::NodeHandle nhp,
                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                            boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                            double ctrl_loop_du)
    {
      nh_ = nh;
      nhp_ = nhp;
      motor_info_pub_ = nh_.advertise<spinal::PwmInfo>("motor_info", 10);
      uav_info_pub_ = nh_.advertise<spinal::UavInfo>("uav_info", 10);

      robot_model_ = robot_model;
      estimator_ = estimator;
      navigator_ = navigator;

      ctrl_loop_du_ = ctrl_loop_du;

      motor_num_ = robot_model->getRotorNum();
      estimate_mode_ = estimator_->getEstimateMode();

      getParam<bool>(nhp_, "param_verbose", param_verbose_, false);
      getParam<int>(nh_, "uav_model", uav_model_, 0); //0: DRONE

      ros::NodeHandle control_nh(nh_, "controller");
      getParam<bool>(control_nh, "control_verbose", control_verbose_, false);

      ros::NodeHandle motor_nh(nh_, "motor_info");
      getParam<double>(motor_nh, "max_pwm", max_pwm_, 0.0);
      getParam<double>(motor_nh, "min_pwm", min_pwm_, 0.0);
      getParam<double>(motor_nh, "min_thrust", min_thrust_, 0.0);
      getParam<double>(motor_nh, "force_landing_thrust", force_landing_thrust_, 0.0);
      getParam<double>(motor_nh, "m_f_rate", m_f_rate_, 0.0);
      getParam<int>(motor_nh, "pwm_conversion_mode", pwm_conversion_mode_, -1);

      int vel_ref_num;
      getParam<int>(motor_nh, "vel_ref_num", vel_ref_num, 0);
      motor_info_.resize(vel_ref_num);
      for(int i = 0; i < vel_ref_num; i++)
        {
          std::stringstream ss;
          ss << i + 1;
          double val;
          ros::NodeHandle nh(motor_nh, "ref" + ss.str());
          getParam<double>(nh, "voltage", val, 0);
          motor_info_[i].voltage = val;
          nh.param("max_thrust", val, 0.0);
          motor_info_[i].max_thrust = val;

          /* hardcode: up to 4 dimension */
          for(int j = 0; j < 5; j++)
            {
              std::stringstream ss2;
              ss2 << j;
              getParam<double>(nh, "polynominal" + ss2.str(), val, 0);
              motor_info_[i].polynominal[j] = val;
            }
        }
    }

    virtual bool update()
    {
      if(navigator_->getNaviState() == aerial_robot_navigation::START_STATE) activate();
      if(navigator_->getNaviState() == aerial_robot_navigation::ARM_OFF_STATE && control_timestamp_ > 0)
        {
          reset();
        }

      if (control_timestamp_ < 0)
        {
          if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
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
      if(ros::Time::now().toSec() - activate_timestamp_  > 0.1)
        {
          /* send motor and uav info to uav, about 10Hz */
          spinal::PwmInfo motor_info_msg;
          motor_info_msg.max_pwm = max_pwm_;
          motor_info_msg.min_pwm = min_pwm_;
          motor_info_msg.min_thrust = min_thrust_;
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

          activate_timestamp_ = ros::Time::now().toSec();

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

    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator_;

    double activate_timestamp_;

    double ctrl_loop_du_;
    double control_timestamp_;
    int motor_num_;
    int uav_model_;

    double m_f_rate_;
    double max_pwm_, min_pwm_;
    double min_thrust_;
    std::vector<spinal::MotorInfo> motor_info_;

    double force_landing_thrust_; //pwm
    int pwm_conversion_mode_;

    int estimate_mode_;
    bool param_verbose_;
    bool control_verbose_;

    template<class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
    {
      nh.param<T>(param_name, param, default_value);

      if(param_verbose_)
        ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
    }
  };

};
