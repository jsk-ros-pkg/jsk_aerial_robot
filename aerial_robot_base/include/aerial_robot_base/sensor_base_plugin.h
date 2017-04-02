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

#ifndef SENOR_BASE_PLUGIN_H_
#define SENOR_BASE_PLUGIN_H_

/* ros */
#include <ros/ros.h>

/* estimation class */
#include <aerial_robot_base/basic_state_estimation.h>

/* filter */
#include <kalman_filter/kf_base_plugin.h>
#include <kalman_filter/digital_filter.h>

/* algebra */
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/transform_listener.h>

/* ros msg */
#include <aerial_robot_msgs/BoolFlag.h>

/* utils */
#include <iostream>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class SensorBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)  = 0;
    virtual ~SensorBase(){}

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Timer  health_check_timer_;
    ros::ServiceServer estimate_flag_service_;
    BasicEstimator* estimator_;
    int estimate_mode_;

    bool simulation_;
    bool param_verbose_;
    bool debug_verbose_;

    //vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors_;

    double sensor_hz_; // hz  of the sensor
    vector<int> estimate_indices_; // the fuser_egomation index
    vector<int> experiment_indices_; // the fuser_experiment indices

    int estimate_flag_;

    /* health check */
    vector<bool> health_;
    vector<double> health_stamp_;
    double health_check_rate_;
    double health_timeout_;
    int unhealth_level_;


    SensorBase(){}

    inline bool getFuserActivate(uint8_t mode)
    {
      return (estimate_mode_ & (1 << mode));
    }

    void baseParamInit(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      estimator_ = estimator;

      nh_ = ros::NodeHandle(nh, sensor_name);
      nhp_ = ros::NodeHandle(nhp, sensor_name);

      estimate_flag_ = true;
      sensor_hz_ = 0;
      health_.resize(1, false);
      health_stamp_.resize(1, ros::Time::now().toSec());

      estimate_flag_service_ = nh_.advertiseService("estimate_flag", &SensorBase::estimateFlag, this);

      string ns = nhp_.getNamespace();
      ROS_WARN("load sensor plugin %s:", ns.c_str());
      if (!nhp_.getParam ("estimate_mode", estimate_mode_))
        ROS_ERROR("%s, can not get param about estimate mode", ns.c_str());
      cout << ns.c_str() << ",  estimate mode  is " << estimate_mode_ << endl;

      nhp_.param("health_timeout", health_timeout_, 0.5);
      nhp_.param("unhealth_level", unhealth_level_, 0);
      nhp_.param("health_check_rate", health_check_rate_, 100.0);

      ros::NodeHandle nh_global("~");
      nh_global.param("simulation", simulation_, false);
      cout << nh_global.getNamespace() << ",  simulaiton  is " << simulation_ << endl;
      nh_global.param("param_verbose", param_verbose_, false);
      nh_global.param("debug_verbose", debug_verbose_, false);

      health_check_timer_ = nhp_.createTimer(ros::Duration(1.0 / health_check_rate_), &SensorBase::healthCheck,this);
    }

    virtual void estimateProcess(){};

    /* check whether we get sensor data */
    void healthCheck(const ros::TimerEvent & e)
    {
      /* this will call only once, no recovery */
      for(int i = 0; i < health_.size(); i++)
        {
          if(ros::Time::now().toSec() - health_stamp_[i] > health_timeout_ && health_[i] && !simulation_)
            {
              ROS_ERROR("[%s, chan%d]: can not get fresh sensor data for %f[sec]", nhp_.getNamespace().c_str(), i, ros::Time::now().toSec() - health_stamp_[i]);
              /* TODO: the solution to unhealth should be more clever */
              estimator_->setUnhealthLevel(unhealth_level_);
              health_[i] = false;
            }
        }
    }

    bool estimateFlag(aerial_robot_msgs::BoolFlag::Request  &req,
                      aerial_robot_msgs::BoolFlag::Response &res)
    {
      string ns = nhp_.getNamespace();
      estimate_flag_ = req.flag;
      ROS_INFO("%s: %s", ns.c_str(), estimate_flag_?string("enable the estimate flag").c_str():string("disable the estimate flag").c_str());
      return true;
    }

    void setHealthChanNum(uint8_t chan_num)
    {
      assert(chan_num > 0);

      health_.resize(chan_num, false);
      health_stamp_.resize(chan_num, ros::Time::now().toSec());
    }

    void updateHealthStamp(double stamp, uint8_t chan = 0)
    {
      if(!health_[chan])
        {
          health_[chan] = true;
          ROS_WARN("%s: get sensor data, du: %f", nhp_.getNamespace().c_str(), stamp - health_stamp_[chan]);
        }
      health_stamp_[chan] = stamp;
    }

  };

};

#endif
