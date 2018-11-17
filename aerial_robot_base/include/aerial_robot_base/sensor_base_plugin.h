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
#include <tf/LinearMath/Transform.h>

/* ros msg */
#include <aerial_robot_msgs/States.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Transform.h>

/* utils */
#include <iostream>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class SensorBase
  {
  public:
    SensorBase():
      estimate_flag_(true),
      sensor_hz_(0)
    {
      sensor_tf_.setIdentity();
    }

    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      estimator_ = estimator;

      nh_ = ros::NodeHandle(nh, sensor_name);
      nhp_ = ros::NodeHandle(nhp, sensor_name);

      health_.resize(1, false);
      health_stamp_.resize(1, ros::Time::now().toSec());

      estimate_flag_service_ = nh_.advertiseService("estimate_flag", &SensorBase::estimateFlag, this);

      ros::NodeHandle nh_global("~");
      nh_global.param("simulation", simulation_, false);
      if(param_verbose_) cout << nh_global.getNamespace() << ", simulaiton is " << simulation_ << endl;
      nh_global.param("param_verbose", param_verbose_, false);
      nh_global.param("debug_verbose", debug_verbose_, false);

      std::string ns = nhp_.getNamespace();
      ROS_WARN("load sensor plugin %s:", ns.c_str());
      if (!nhp_.getParam ("estimate_mode", estimate_mode_))
        ROS_ERROR("%s, can not get param about estimate mode", ns.c_str());
      if(param_verbose_) cout << ns << ": estimate mode is " << estimate_mode_ << endl;

      nhp_.param("sensor_frame", sensor_frame_, string("sensor_frame"));
      nhp_.param("reference_frame", reference_frame_, string("baselink"));
      nhp_.param("get_sensor_tf", get_sensor_tf_, false);
      nhp_.param("variable_sensor_tf_flag", variable_sensor_tf_flag_, false);

      nhp_.param("health_timeout", health_timeout_, 0.5);
      if(param_verbose_) cout << ns << ": health_timeout is " << health_timeout_ << endl;
      nhp_.param("unhealth_level", unhealth_level_, 0);
      if(param_verbose_) cout << ns << ": unhealth_level is " << unhealth_level_ << endl;
      nhp_.param("health_check_rate", health_check_rate_, 100.0);
      if(param_verbose_) cout << ns << ": health_check_rate_ is " << health_check_rate_ << endl;

      /* time sync for sensor fusion */
      nhp_.param("delay", delay_, 0.0);
      nhp_.param("time_sync", time_sync_, false);
      if(param_verbose_) cout << ns << ": time_sync: " << time_sync_ << ", delay: " << delay_ << endl;

      health_check_timer_ = nhp_.createTimer(ros::Duration(1.0 / health_check_rate_), &SensorBase::healthCheck,this);
    }

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

    bool get_sensor_tf_;
    bool variable_sensor_tf_flag_;
    string sensor_frame_;
    string reference_frame_;

    bool time_sync_;
    double delay_;

    //vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors_;

    double sensor_hz_; // hz  of the sensor
    vector<int> estimate_indices_; // the fuser_egomation index
    vector<int> experiment_indices_; // the fuser_experiment indices

    int estimate_flag_;

    /* the transformation between sensor frame and baselink frame */
    tf::Transform sensor_tf_; /* TODO: this should be in the basic estimation */

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

    bool estimateFlag(std_srvs::SetBool::Request  &req,
                      std_srvs::SetBool::Response &res)
    {
      string ns = nhp_.getNamespace();
      estimate_flag_ = req.data;
      ROS_INFO("%s: %s", ns.c_str(), estimate_flag_?string("enable the estimate flag").c_str():string("disable the estimate flag").c_str());
      return true;
    }

    void setHealthChanNum(uint8_t chan_num)
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
         TODO: for joint or servo system, this should be processed every time,
         maybe KDL kinematics is better, since the tf need 0.x[sec].
      */
      double start_time = ros::Time::now().toSec();

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      geometry_msgs::TransformStamped transformStamped;
      try
        {
          transformStamped = tfBuffer.lookupTransform(reference_frame_, sensor_frame_, ros::Time(0), ros::Duration(1.0));
        }
      catch (tf2::TransformException &ex)
        {
          ROS_ERROR("%s: %s",nhp_.getNamespace().c_str(), ex.what());
          return false;
        }
      tf::transformMsgToTF(transformStamped.transform, sensor_tf_);

      double y, p, r; sensor_tf_.getBasis().getRPY(r, p, y);
      if(!variable_sensor_tf_flag_)
        ROS_INFO("%s: get tf from %s to %s, [%f, %f, %f], [%f, %f, %f], using %f[sec]",
                 nhp_.getNamespace().c_str(), reference_frame_.c_str(), sensor_frame_.c_str(),
                 sensor_tf_.getOrigin().x(), sensor_tf_.getOrigin().y(),
                 sensor_tf_.getOrigin().z(), r, p, y, ros::Time::now().toSec() - start_time);

      get_sensor_tf_ = true;
      return true;
    }
  };

};

#endif
