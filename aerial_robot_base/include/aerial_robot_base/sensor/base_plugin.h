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

/* filter */
#include <kalman_filter/lpf_filter.h>

/* kf plugin */
#include <kalman_filter/kf_base_plugin.h>

/* estimation class */
#include <aerial_robot_base/state_estimation.h>

/* algebra */
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/LinearMath/Transform.h>

/* ros msg */
#include <aerial_robot_msgs/States.h>
#include <std_srvs/SetBool.h>
#include <tf_conversions/tf_kdl.h>

/* utils */
#include <iostream>

using namespace Eigen;
using namespace std;

namespace Status
{
  enum {INACTIVE = 0, INIT = 1, ACTIVE = 2, INVALID = 3,};
}

namespace sensor_plugin
{
  class SensorBase
  {
  public:
    SensorBase(string plugin_name = string("")):
      sensor_hz_(0), plugin_name_(plugin_name), get_sensor_tf_(false)
    {
      sensor_tf_.setIdentity();
      sensor_status_ = Status::INACTIVE;
    }

    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, StateEstimator* estimator, string sensor_name, int index)
    {
      estimator_ = estimator;

      nh_ = ros::NodeHandle(nh, sensor_name);
      nhp_ = ros::NodeHandle(nhp, sensor_name);
      indexed_nh_ = ros::NodeHandle(nh, sensor_name + std::to_string(index));
      indexed_nhp_ = ros::NodeHandle(nhp, sensor_name + std::to_string(index));

      health_.resize(1, false);
      health_stamp_.resize(1, ros::Time::now().toSec());

      set_status_service_ = indexed_nh_.advertiseService("estimate_flag", &SensorBase::setStatusCb, this);

      ros::NodeHandle nh_global("~");
      nh_global.param("simulation", simulation_, false);
      if(param_verbose_) cout << nh_global.getNamespace() << ", simulaiton is " << simulation_ << endl;
      nh_global.param("param_verbose", param_verbose_, false);
      nh_global.param("debug_verbose", debug_verbose_, false);

      ROS_WARN_STREAM("load sensor plugin: " << indexed_nhp_.getNamespace());

      if (!nhp_.getParam ("estimate_mode", estimate_mode_) && !indexed_nhp_.getParam ("estimate_mode", estimate_mode_))
        ROS_ERROR_STREAM(indexed_nhp_.getNamespace() << ", can not get param about estimate mode");

      /* general parameters for the same sensor type */
      getParam<bool>("param_verbose", param_verbose_, param_verbose_);
      getParam<bool>("debug_verbose", debug_verbose_, debug_verbose_);
      getParam<std::string>("sensor_frame", sensor_frame_, "sensor_frame");
      getParam<bool>("variable_sensor_tf_flag", variable_sensor_tf_flag_, false);
      getParam<double>("health_timeout", health_timeout_, 0.5);
      getParam<int>("unhealth_level", unhealth_level_, 0);
      getParam<double>("health_check_rate", health_check_rate_, 100.0);
      getParam<bool>("time_sync", time_sync_, false);
      getParam<double>("delay", delay_, 0.0);

      health_check_timer_ = indexed_nhp_.createTimer(ros::Duration(1.0 / health_check_rate_), &SensorBase::healthCheck,this);
    }

    virtual ~SensorBase(){}

    inline const std::string& getPluginName() const {return plugin_name_;}
    const int getStatus()
    {
      boost::lock_guard<boost::mutex> lock(status_mutex_);
      return sensor_status_;
    }

    void setStatus(const int status)
    {
      boost::lock_guard<boost::mutex> lock(status_mutex_);
      prev_status_ = sensor_status_;
      sensor_status_ = status;
    }

  protected:

    ros::NodeHandle nh_, nhp_; //node handle for same sensor type (e.g. imu, gps, vo, alt)
    ros::NodeHandle indexed_nh_, indexed_nhp_; //node handle for indexed sensor handler (e.g. imu1, imu2)
    ros::Timer  health_check_timer_;
    ros::ServiceServer set_status_service_;
    StateEstimator* estimator_;
    int estimate_mode_;

    bool simulation_;
    bool param_verbose_;
    bool debug_verbose_;

    string plugin_name_;

    bool get_sensor_tf_;
    bool variable_sensor_tf_flag_;

    string sensor_frame_;

    bool time_sync_;
    double delay_;
    double curr_timestamp_;
    double prev_timestamp_;
    double sensor_hz_; // hz  of the sensor
    vector<int> estimate_indices_; // the fuser_egomation index
    vector<int> experiment_indices_; // the fuser_experiment indices

    /* the transformation between sensor frame and baselink frame */
    tf::Transform sensor_tf_;

    /* status */
    int sensor_status_;
    int prev_status_;
    boost::mutex status_mutex_;

    /* health check */
    vector<bool> health_;
    vector<double> health_stamp_;
    double health_check_rate_;
    double health_timeout_;
    int unhealth_level_;


    inline const bool getFuserActivate(uint8_t mode) const
    {
      return (estimate_mode_ & (1 << mode));
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

    bool setStatusCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
    {
      if(sensor_status_ == Status::INVALID && req.data)
        {
          sensor_status_ == Status::INACTIVE;
          ROS_INFO_STREAM(nhp_.getNamespace() << "enable the estimate flag");
        }
      if(!req.data)
        {
          sensor_status_ = Status::INVALID;
          ROS_INFO_STREAM(nhp_.getNamespace() << "disable the estimate flag");
        }
      return true;
    }

    void setHealthChanNum(const uint8_t& chan_num)
    {
      assert(chan_num > 0);

      health_.resize(chan_num, false);
      health_stamp_.resize(chan_num, ros::Time::now().toSec());
    }

    void updateHealthStamp(uint8_t chan = 0)
    {
      if(!health_[chan])
        {
          health_[chan] = true;
          ROS_WARN("%s: get sensor data, du: %f", nhp_.getNamespace().c_str(), ros::Time::now().toSec() - health_stamp_[chan]);
        }
      health_stamp_[chan] = ros::Time::now().toSec();
    }

    inline const tf::Transform& getBaseLink2SensorTransform() const { return sensor_tf_; }

    bool updateBaseLink2SensorTransform()
    {
      /* get transform from baselink to sensor frame */
      if(!variable_sensor_tf_flag_ && get_sensor_tf_) return true;

      /*
        for joint or servo system, this should be processed every time,
        therefore kinematics based on kinematics is better, since the tf need 0.x[sec].
      */
      const auto& segments_tf =  estimator_->getSegmentsTf();

      if(segments_tf.empty())
        {
          if(get_sensor_tf_) ROS_ERROR("the segment tf is empty after init phase");

          ROS_DEBUG_STREAM("segment tf is empty");
          return false;
        }

      if(segments_tf.find(sensor_frame_) == segments_tf.end())
        {
          ROS_ERROR_THROTTLE(0.5, "can not find %s in kinematics model", sensor_frame_.c_str());
          return false;
        }

      try
        {
          tf::transformKDLToTF(segments_tf.at(estimator_->getBaselinkName()).Inverse() * segments_tf.at(sensor_frame_), sensor_tf_);
        }
      catch (...)
        {
          ROS_ERROR("Bug: can not find %s in spite of segments_tf.find is true", sensor_frame_.c_str());
        }

      double y, p, r; sensor_tf_.getBasis().getRPY(r, p, y);
      if(!variable_sensor_tf_flag_)
        ROS_INFO("%s: get tf from %s to %s, [%f, %f, %f], [%f, %f, %f]",
                 nhp_.getNamespace().c_str(),
                 estimator_->getBaselinkName().c_str(), sensor_frame_.c_str(),
                 sensor_tf_.getOrigin().x(), sensor_tf_.getOrigin().y(),
                 sensor_tf_.getOrigin().z(), r, p, y);

      get_sensor_tf_ = true;
      return true;
    }

    template<class T> void getParam(std::string param_name, T& param, T default_value)
    {
      nhp_.param<T>(param_name, param, default_value);
      if(indexed_nhp_.hasParam(param_name)) indexed_nhp_.getParam(param_name, param);

      if(param_verbose_)
        ROS_INFO_STREAM("[" << indexed_nhp_.getNamespace() << "] " << param_name << ": " << param);
    }
  };

};

#endif
