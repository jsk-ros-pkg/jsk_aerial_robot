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

void RigidEstimator::statePublish()
{
  aerial_robot_base::States full_state;
  full_state.header.stamp = ros::Time::now();

  for(int axis = 0; axis < State::TOTAL_NUM; axis++)
    {
      aerial_robot_base::State r_state;
      AxisState state = getState(axis);

      switch(axis)
        {
        case State::X_COG:
          r_state.id = "x_cog";
          break;
        case State::Y_COG:
          r_state.id = "y_cog";
          break;
        case State::Z_COG:
          r_state.id = "z_cog";
          break;
        case State::X_BASE:
          r_state.id = "x_b";
          break;
        case State::Y_BASE:
          r_state.id = "y_b";
          break;
        case State::Z_BASE:
          r_state.id = "z_b";
          break;
        case State::ROLL_COG:
          r_state.id = "roll_cog";
          break;
        case State::PITCH_COG:
          r_state.id = "pitch_cog";
          break;
        case State::YAW_COG:
          r_state.id = "yaw_cog";
          break;
        case State::ROLL_BASE:
          r_state.id = "roll_b";
          break;
        case State::PITCH_BASE:
          r_state.id = "pitch_b";
          break;
        case State::YAW_BASE:
          r_state.id = "yaw_b";
          break;
        default:
          break;
        }
      r_state.state.resize(3);
      for(int mode = 0; mode < 3; mode++)
        tf::vector3TFToMsg(state[mode].second, r_state.state[mode]);

      full_state.states.push_back(r_state);
    }
  full_state_pub_.publish(full_state);

  nav_msgs::Odometry odom_state;
  odom_state.header.stamp = ros::Time::now();
  odom_state.header.frame_id = std::string("/nav");

  /* Baselink */
  /* Rotation */
  tf::Quaternion q; getOrientation(Frame::BASELINK, estimate_mode_).getRotation(q);
  tf::quaternionTFToMsg(q, odom_state.pose.pose.orientation);
  tf::vector3TFToMsg(getAngularVel(Frame::BASELINK, estimate_mode_), odom_state.twist.twist.angular);

  /* Translation */
  odom_state.child_frame_id = std::string("/baselink");
  tf::pointTFToMsg(getPos(Frame::BASELINK, estimate_mode_), odom_state.pose.pose.position);
  tf::vector3TFToMsg(getVel(Frame::BASELINK, estimate_mode_), odom_state.twist.twist.linear);
  baselink_odom_pub_.publish(odom_state);


  /* COG */
  /* Rotation */
  getOrientation(Frame::BASELINK, estimate_mode_).getRotation(q);
  tf::quaternionTFToMsg(q, odom_state.pose.pose.orientation);
  tf::vector3TFToMsg(getAngularVel(Frame::COG, estimate_mode_), odom_state.twist.twist.angular);
  /* Translation */
  odom_state.child_frame_id = std::string("/cog");
  tf::pointTFToMsg(getPos(Frame::COG, estimate_mode_), odom_state.pose.pose.position);
  tf::vector3TFToMsg(getVel(Frame::COG, estimate_mode_), odom_state.twist.twist.linear);
  cog_odom_pub_.publish(odom_state);

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

  sensor_fusion_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_plugin::KalmanFilter>("kalman_filter", "kf_plugin::KalmanFilter"));

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

              boost::shared_ptr<kf_plugin::KalmanFilter> plugin_ptr = sensor_fusion_loader_ptr_->createInstance(name);
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
