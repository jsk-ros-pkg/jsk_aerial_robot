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
#include <spinal/PwmInfos.h>
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
      motor_info_pub_ = nh_.advertise<spinal::PwmInfos>("motor_info", 10);
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
      motor_types_.resize(0);
      int motor_type_max = 0;
      if(motor_nh.hasParam("motor_types"))
        {
          motor_nh.getParam("motor_types", motor_types_);
          for(int i = 0; i < motor_types_.size(); i++)
            if(motor_types_[i] > motor_type_max) motor_type_max = motor_types_[i]; // assume motor type is started from 0 and continuous
        }

      max_pwm_.resize(motor_type_max + 1, 0.0);
      min_pwm_.resize(motor_type_max + 1, 0.0);
      min_thrust_.resize(motor_type_max + 1, 0.0);
      motor_info_.resize(motor_type_max + 1);
      force_landing_thrust_.resize(motor_type_max + 1, 0.0);
      pwm_conversion_mode_.resize(motor_type_max + 1, spinal::MotorInfo::POLYNOMINAL_MODE);

      for(int i = 0; i < motor_type_max + 1; i++)
        {
          std::stringstream ss1;
          if(i != 0) ss1 << i; // use "motor_info" for type 0, "motor_info1" for type 1, etc.
          ros::NodeHandle motor_type_nh(nh_, "motor_info" + ss1.str());

          getParam<double>(motor_type_nh, "max_pwm", max_pwm_[i], 0.0);
          getParam<double>(motor_type_nh, "min_pwm", min_pwm_[i], 0.0);
          getParam<double>(motor_type_nh, "min_thrust", min_thrust_[i], 0.0);
          getParam<double>(motor_type_nh, "force_landing_thrust", force_landing_thrust_[i], 0.0);
          getParam<int>(motor_type_nh, "pwm_conversion_mode", pwm_conversion_mode_[i], -1);

          int vel_ref_num;
          getParam<int>(motor_type_nh, "vel_ref_num", vel_ref_num, 0);
          motor_info_.at(i).resize(vel_ref_num);

          for(int j = 0; j < vel_ref_num; j++)
            {
              std::stringstream ss2;
              ss2 << j + 1; // use "ref1", "ref2", etc.
              double val;
              ros::NodeHandle nh(motor_type_nh, "ref" + ss2.str());
              getParam<double>(nh, "voltage", val, 0);
              motor_info_[i][j].voltage = val;
              nh.param("max_thrust", val, 0.0);
              motor_info_[i][j].max_thrust = val;

              /* hardcode: up to 4 dimension */
              for(int k = 0; k < 5; k++)
                {
                  std::stringstream ss3;
                  ss3 << k;
                  getParam<double>(nh, "polynominal" + ss3.str(), val, 0);
                  motor_info_[i][j].polynominal[k] = val;
                }
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
          spinal::PwmInfos pwm_infos;
          for(int i = 0; i < motor_types_.size(); i++)
            pwm_infos.motor_types.push_back(motor_types_[i]);
          for(int i = 0; i < motor_info_.size(); i++)
            {
              spinal::PwmInfo motor_info_msg;
              motor_info_msg.max_pwm = max_pwm_[i];
              motor_info_msg.min_pwm = min_pwm_[i];
              motor_info_msg.min_thrust = min_thrust_[i];
              motor_info_msg.force_landing_thrust = force_landing_thrust_[i];
              motor_info_msg.pwm_conversion_mode = pwm_conversion_mode_[i];
              motor_info_msg.motor_info.resize(0);
              for(int j = 0; j < motor_info_.at(i).size(); j++)
                motor_info_msg.motor_info.push_back(motor_info_[i][j]);
              pwm_infos.pwm_infos.push_back(motor_info_msg);
            }
          motor_info_pub_.publish(pwm_infos);

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
    std::vector<int> motor_types_;
    std::vector<double> max_pwm_, min_pwm_;
    std::vector<double> min_thrust_;
    std::vector<std::vector<spinal::MotorInfo>> motor_info_;
    std::vector<double> force_landing_thrust_; //pwm
    std::vector<int> pwm_conversion_mode_;

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
