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

#include "aerial_robot_base/state_estimation.h"

RigidEstimator::RigidEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  BasicEstimator(nh, nh_private)
{
  rosParamInit();

}

RigidEstimator::~RigidEstimator() {}

void RigidEstimator::tfPublish()
{
  statesBroadcast();
}


void RigidEstimator::statesBroadcast()
{
  aerial_robot_base::States full_state;
  full_state.header.stamp = ros::Time::now();

  for(int axis = 0; axis < STATE_NUM - 3; axis++)
    {
      aerial_robot_base::State r_state;
      AxisState state = getState(axis);

      switch(axis)
        {
        case X_W:
          r_state.id = "x";
          break;
        case Y_W:
          r_state.id = "y";
          break;
        case Z_W:
          r_state.id = "z";
          break;
        case ROLL_W:
          r_state.id = "roll";
          break;
        case PITCH_W:
          r_state.id = "pitch";
          break;
        case YAW_W:
          r_state.id = "yaw";
          break;
        default:
          break;
        }

      r_state.state.resize(3);
      for(int mode = 0; mode < 3; mode++)
        tf::vector3TFToMsg(state[mode].second, r_state.state[mode]);

      if(axis == X_W || axis == Y_W || axis == YAW_W)
        {
          AxisState state_temp;
          if(axis == X_W) state_temp = getState(X_B);
          if(axis == Y_W) state_temp = getState(Y_B);
          if(axis == YAW_W) state_temp = getState(YAW_W_B);

          for(int mode = 0; mode < 3; mode++)
            {
              r_state.reserves.push_back(state_temp[mode].second[0]);
              r_state.reserves.push_back(state_temp[mode].second[1]);
            }
        }

      full_state.states.push_back(r_state);
    }
  full_state_pub_.publish(full_state);

  nav_msgs::Odometry odom_state;
  odom_state.header.stamp = ros::Time::now();
  odom_state.header.frame_id = std::string("/nav");
  odom_state.child_frame_id = std::string("/root_link");

  tf::Point pos;
  tf::Vector3 vel;
  tf::Quaternion q;
  tf::Vector3 omega;

  pos.setValue(getState(X_W)[estimate_mode_].second.x(),
               getState(Y_W)[estimate_mode_].second.x(),
               getState(Z_W)[estimate_mode_].second.x());
  tf::pointTFToMsg(pos, odom_state.pose.pose.position);

  vel.setValue(getState(X_W)[estimate_mode_].second.y(),
               getState(Y_W)[estimate_mode_].second.y(),
               getState(Z_W)[estimate_mode_].second.y());
  tf::vector3TFToMsg(vel, odom_state.twist.twist.linear);

  /* cog frame */
  q.setRPY(getState(ROLL_W_B)[estimate_mode_].second.x(),
           getState(PITCH_W_B)[estimate_mode_].second.x(),
           getState(YAW_W_B)[estimate_mode_].second.x());
  tf::quaternionTFToMsg(q, odom_state.pose.pose.orientation);

  omega.setValue(getState(ROLL_W_B)[estimate_mode_].second.y(),
                 getState(PITCH_W_B)[estimate_mode_].second.y(),
                 getState(YAW_W_B)[estimate_mode_].second.y());
  tf::vector3TFToMsg(omega, odom_state.twist.twist.angular);

  odom_state_pub_.publish(odom_state);
}


bool RigidEstimator::pattern_match(std::string &pl, std::string &pl_candidate)
{
  int cmp = fnmatch(pl.c_str(), pl_candidate.c_str(), FNM_CASEFOLD);
  if (cmp == 0)
    return true;
  else if (cmp != FNM_NOMATCH) {
    // never see that, i think that it is fatal error.
    ROS_FATAL("Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
              pl.c_str(), pl_candidate.c_str(), cmp);
    ros::shutdown();
  }
  return false;
}

void RigidEstimator::rosParamInit()
{
  std::string ns = nhp_.getNamespace();

  sensor_fusion_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>("kalman_filter", "kf_base_plugin::KalmanFilter"));

  /* kalman filter egomotion plugin initialization for 0: egomotion, 1: experiment */
  for (int i = 0; i < 2; i++)
    {
      /* kalman filter egomotion plugin list */
      ros::V_string pl_list{};
      string prefix;
      if(i == EGOMOTION_ESTIMATE) prefix = string("egomotion");
      else if(i == EXPERIMENT_ESTIMATE) prefix = string("experiment");

      nhp_.getParam(prefix + "_list", pl_list);

      for (auto &pl_name : pl_list)
        {
          for (auto &name : sensor_fusion_loader_ptr_->getDeclaredClasses())
            {
              if(!pattern_match(pl_name, name)) continue;

              std::stringstream fuser_no;
              fuser_no << fuser_[i].size() + 1;

              int fuser_id;
              string fuser_name;

              if (!nhp_.getParam ("fuser_" + prefix + "_id" + fuser_no.str(), fuser_id))
                ROS_ERROR("%s, no param in fuser %s id", prefix.c_str(), fuser_no.str().c_str());
              if(param_verbose_) cout << "fuser_"  << prefix << "_id" << fuser_no.str() << " is " << fuser_id << endl;

              if (!nhp_.getParam ("fuser_" + prefix + "_name" + fuser_no.str(), fuser_name))
                ROS_ERROR("%s, no param in fuser %s name", prefix.c_str(), fuser_no.str().c_str());
              if(param_verbose_) cout << "fuser_"  << prefix << "_name" << fuser_no.str() << " is " << fuser_name << endl;

              boost::shared_ptr<kf_base_plugin::KalmanFilter> plugin_ptr = sensor_fusion_loader_ptr_->createInstance(name);
              plugin_ptr->initialize(nh_, fuser_name, fuser_id);
              fuser_[i].push_back(make_pair(name, plugin_ptr));

              break;
            }
        }
    }

  sensor_plugin_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<sensor_plugin::SensorBase> >( new pluginlib::ClassLoader<sensor_plugin::SensorBase>("aerial_robot_base", "sensor_plugin::SensorBase"));

  ros::V_string sensor_list{};
  nhp_.getParam("sensor_list", sensor_list);

  for (auto &sensor_plugin_name : sensor_list)
    {
      for (auto &name : sensor_plugin_ptr_->getDeclaredClasses())
        {
          if(!pattern_match(sensor_plugin_name, name)) continue;

          sensors_.push_back(sensor_plugin_ptr_->createInstance(name));
          break;
        }
    }

  /* initilaize in the same time */
  for(size_t i = 0; i < sensors_.size(); i++)
    sensors_[i]->initialize(nh_, ros::NodeHandle(""), this, sensor_list[i]);
}
