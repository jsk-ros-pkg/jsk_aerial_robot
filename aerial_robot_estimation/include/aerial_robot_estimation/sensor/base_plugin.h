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

#pragma once

#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_msgs/States.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <kalman_filter/lpf_filter.h>
#include <kalman_filter/kf_base_plugin.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/LinearMath/Transform.h>
#include <tf_conversions/tf_kdl.h>

using namespace Eigen;
using namespace std;

namespace Status
{
  enum {INACTIVE = 0, INIT = 1, ACTIVE = 2, INVALID = 3, RESET = 4};
}

namespace sensor_plugin
{
  class SensorBase
  {
  public:
    SensorBase(): sensor_hz_(0), get_sensor_tf_(false)
    {
      sensor_tf_.setIdentity();
      sensor_status_ = Status::INACTIVE;
    }

    virtual void initialize(ros::NodeHandle nh,
                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                            string sensor_name, int index)
    {
      estimator_ = estimator;
      robot_model_ = robot_model;

      nh_ = nh;
      nhp_ = ros::NodeHandle(nh_, sensor_name);
      indexed_nhp_ = ros::NodeHandle(nh_, sensor_name + std::to_string(index));

      health_.resize(1, false);
      health_stamp_.resize(1, ros::Time::now().toSec());

      sensor_name_ = sensor_name.substr(sensor_name.rfind("/") + 1);
      state_pub_ = nh_.advertise<aerial_robot_msgs::States>("kf/" + sensor_name_ + std::to_string(index) + "/data", 10);
      set_status_service_ = indexed_nhp_.advertiseService("estimate_flag", &SensorBase::setStatusCb, this);
      reset_service_ = indexed_nhp_.advertiseService("reset", &SensorBase::resetCb, this);

      ROS_INFO_STREAM("load sensor plugin: " << sensor_name_ + std::to_string(index));

      if (!nhp_.getParam ("estimate_mode", estimate_mode_) && !indexed_nhp_.getParam ("estimate_mode", estimate_mode_))
        ROS_ERROR_STREAM(indexed_nhp_.getNamespace() << ", can not get param about estimate mode");

      /* general parameters for the same sensor type */
      getParam<bool>("param_verbose", param_verbose_, false);
      getParam<bool>("debug_verbose", debug_verbose_, false);
      getParam<std::string>("sensor_frame", sensor_frame_, "sensor_frame");
      getParam<bool>("variable_sensor_tf_flag", variable_sensor_tf_flag_, false);
      getParam<double>("reset_duration", reset_duration_, 1.0);
      getParam<double>("health_timeout", health_timeout_, 0.5);
      getParam<int>("unhealth_level", unhealth_level_, 0);
      getParam<double>("health_check_rate", health_check_rate_, 100.0);
      getParam<bool>("time_sync", time_sync_, false);
      getParam<double>("delay", delay_, 0.0);

      health_check_timer_ = indexed_nhp_.createTimer(ros::Duration(1.0 / health_check_rate_), &SensorBase::healthCheck,this);
    }

    virtual ~SensorBase(){}

    inline const std::string& getSensorName() const {return sensor_name_;}
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

    virtual bool reset()
    {
      return true;
    }

    virtual void changeStatus(bool flag)
    {
      if(sensor_status_ == Status::INVALID && flag)
        {
          sensor_status_ = Status::INACTIVE;
          ROS_INFO_STREAM(indexed_nhp_.getNamespace() << ", set to inactive");
        }
      if(!flag)
        {
          sensor_status_ = Status::INVALID;
          ROS_INFO_STREAM(indexed_nhp_.getNamespace() << ", set to invalid");
        }
    }

  protected:

    ros::NodeHandle nh_, nhp_;
    ros::NodeHandle indexed_nhp_; //node handle for indexed sensor handler (e.g. imu1, imu2)
    ros::Publisher state_pub_;
    ros::Timer  health_check_timer_;
    ros::ServiceServer set_status_service_;
    ros::ServiceServer reset_service_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
    int estimate_mode_;

    //bool simulation_;
    bool param_verbose_;
    bool debug_verbose_;

    string sensor_name_;

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
    boost::mutex health_check_mutex_;

    /* health check */
    double reset_stamp_;
    double reset_duration_;
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
      boost::lock_guard<boost::mutex> lock(health_check_mutex_);

      /* this will call only once, no recovery */
      for(int i = 0; i < health_.size(); i++)
        {
          if(ros::Time::now().toSec() - health_stamp_.at(i) > health_timeout_ && health_.at(i)) //  && !simulation_
            {
              ROS_ERROR("[%s, chan%d]: can not get fresh sensor data for %f[sec]", indexed_nhp_.getNamespace().c_str(), i, ros::Time::now().toSec() - health_stamp_.at(i));
              /* TODO: the solution to unhealth should be more clever */
              estimator_->setUnhealthLevel(unhealth_level_);

              health_.at(i) = false;
            }
        }
    }

    bool resetCb(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
    {
      ROS_INFO("reset sensor plugin %s from rosservice server", sensor_name_.c_str());
      reset();
      return true;
    }

    bool setStatusCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
    {
      changeStatus(req.data);

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
      boost::lock_guard<boost::mutex> lock(health_check_mutex_);

      if(!health_[chan])
        {
          health_[chan] = true;
          ROS_WARN("%s: get sensor data, du: %f", indexed_nhp_.getNamespace().c_str(), ros::Time::now().toSec() - health_stamp_[chan]);
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
      const auto segments_tf =  robot_model_->getSegmentsTf();

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
          tf::transformKDLToTF(segments_tf.at(robot_model_->getBaselinkName()).Inverse() * segments_tf.at(sensor_frame_), sensor_tf_);
        }
      catch (...)
        {
          ROS_ERROR("Bug: can not find %s in spite of segments_tf.find is true", sensor_frame_.c_str());
        }

      double y, p, r; sensor_tf_.getBasis().getRPY(r, p, y);
      if(!variable_sensor_tf_flag_)
        ROS_INFO("%s: get tf from %s to %s, [%f, %f, %f], [%f, %f, %f]",
                 indexed_nhp_.getNamespace().c_str(),
                 robot_model_->getBaselinkName().c_str(), sensor_frame_.c_str(),
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
