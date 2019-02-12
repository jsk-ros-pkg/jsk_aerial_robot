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

/* sensor plugin */
#include <aerial_robot_base/sensor/base_plugin.h>

StateEstimator::StateEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh, "estimator"),
    nhp_(nh_private, "estimator"),
    sensor_fusion_flag_(false),
    qu_size_(0),
    flying_flag_(false),
    landing_mode_flag_(false),
    landed_flag_(false),
    un_descend_flag_(false),
    landing_height_(0),
    force_att_control_flag_(false),
    mass_(0)
{
  fuser_[0].resize(0);
  fuser_[1].resize(0);

  for(int i = 0; i < State::TOTAL_NUM; i ++)
    {
      for(int j = 0; j < 3; j++)
        {
          state_[i][j].first = 0;
          state_[i][j].second = tf::Vector3(0, 0, 0);
        }
    }

  /* initialize the multilink kinematics */
  kinematics_model_ = boost::shared_ptr<aerial_robot_model::RobotModel>(new aerial_robot_model::RobotModel(true));
  baselink_name_ = kinematics_model_->getBaselinkName();
  cog2baselink_transform_.setIdentity();

  /* TODO: represented sensors unhealth level */
  unhealth_level_ = 0;

  rosParamInit();

  baselink_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/uav/baselink/odom", 1);
  cog_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/uav/cog/odom", 1);
  full_state_pub_ = nh_.advertise<aerial_robot_msgs::States>("/uav/full_state", 1);

  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &StateEstimator::jointStateCallback, this);
  cog2baselink_transform_sub_ = nh_.subscribe(cog2baselink_transform_sub_name_, 5, &StateEstimator::transformCallback, this);

  nhp_.param ("update_rate", update_rate_, 100.0); //100Hz
  update_thread_ = boost::thread(boost::bind(&StateEstimator::update, this));
}

void StateEstimator::statePublish()
{
  aerial_robot_msgs::States full_state;
  full_state.header.stamp = ros::Time::now();

  for(int axis = 0; axis < State::TOTAL_NUM; axis++)
    {
      aerial_robot_msgs::State r_state;
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
  getOrientation(Frame::COG, estimate_mode_).getRotation(q);
  tf::quaternionTFToMsg(q, odom_state.pose.pose.orientation);
  tf::vector3TFToMsg(getAngularVel(Frame::COG, estimate_mode_), odom_state.twist.twist.angular);
  /* Translation */
  odom_state.child_frame_id = std::string("/cog");
  tf::pointTFToMsg(getPos(Frame::COG, estimate_mode_), odom_state.pose.pose.position);
  tf::vector3TFToMsg(getVel(Frame::COG, estimate_mode_), odom_state.twist.twist.linear);
  cog_odom_pub_.publish(odom_state);

  tf::Transform wrold2cog_transform_;
  tf::poseMsgToTF(odom_state.pose.pose, wrold2cog_transform_);
  br_.sendTransform(tf::StampedTransform(wrold2cog_transform_.inverse(), ros::Time::now(), "cog", "world"));
}


bool StateEstimator::pattern_match(std::string &pl, std::string &pl_candidate)
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

void StateEstimator::rosParamInit()
{
  ros::NodeHandle global_nh("~");
  global_nh.param ("param_verbose", param_verbose_, true);

  nhp_.param ("estimate_mode", estimate_mode_, 0); //EGOMOTION_ESTIMATE: 0
  nhp_.param("cog2baselink_transform_sub_name", cog2baselink_transform_sub_name_, std::string("/cog2baselink"));
  ROS_WARN("estimate_mode is %s", (estimate_mode_ == EGOMOTION_ESTIMATE)?string("EGOMOTION_ESTIMATE").c_str():((estimate_mode_ == EXPERIMENT_ESTIMATE)?string("EXPERIMENT_ESTIMATE").c_str():((estimate_mode_ == GROUND_TRUTH)?string("GROUND_TRUTH").c_str():string("WRONG_MODE").c_str())));

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
              if(param_verbose_) cout << ns << ": fuser_"  << prefix << "_id" << fuser_no.str() << " is " << fuser_id << endl;

              if (!nhp_.getParam ("fuser_" + prefix + "_name" + fuser_no.str(), fuser_name))
                ROS_ERROR("%s, no param in fuser %s name", prefix.c_str(), fuser_no.str().c_str());
              if(param_verbose_) cout << ns << ": fuser_"  << prefix << "_name" << fuser_no.str() << " is " << fuser_name << endl;

              boost::shared_ptr<kf_plugin::KalmanFilter> plugin_ptr = sensor_fusion_loader_ptr_->createInstance(name);
              plugin_ptr->initialize(fuser_name, fuser_id);
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

          if(sensors_.back()->getPluginName() == std::string(""))
            {
              ROS_ERROR("invalid sensor plugin");
              sensors_.pop_back();
            }
          else
            {
              if(sensors_.back()->getPluginName() == std::string("imu"))
                imu_handler_ = sensors_.back();
              else if(sensors_.back()->getPluginName() == std::string("vo"))
                vo_handler_ = sensors_.back();
              else if(sensors_.back()->getPluginName() == std::string("gps"))
                gps_handler_ = sensors_.back();
              else if(sensors_.back()->getPluginName() == std::string("alt"))
                alt_handler_ = sensors_.back();
            }
          break;
        }
    }

  /* initilaize in the same time */
  for(size_t i = 0; i < sensors_.size(); i++)
    sensors_[i]->initialize(nh_, ros::NodeHandle(""), this, sensor_list[i]);
}
