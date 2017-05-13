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

#ifndef BASIC_STATE_ESTIMATION_H_
#define BASIC_STATE_ESTIMATION_H_

/* ros */
#include <ros/ros.h>

/* kf plugin */
#include <pluginlib/class_loader.h>
#include <kalman_filter/kf_base_plugin.h>

/* ros msg */
#include <aerial_robot_base/States.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8.h>

/* util */
#include <assert.h>
#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include <array>

using namespace std;

/* int: state_estimate_status (EGOMOTION_ESTIMATE or Experiment or ground truth )
   tf::Vector3:  [x, dx, ddx]
   3:  0: egomotion estiamte, 1: experiment, 2: ground truth
*/
using State3Mode = std::array<tf::Vector3, 3>;
using StateWithStatus = std::pair<int, std::array<tf::Vector3, 3> >;
using SensorFuser = std::vector< std::pair<std::string, boost::shared_ptr<kf_base_plugin::KalmanFilter> > >;

namespace Frame
{
  enum {BODY = 0, COG = 1,};
};

class BasicEstimator
{

public:
  BasicEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh, "estimator"),
      nhp_(nh_private, "estimator"),
      sensor_fusion_flag_(false),
      flying_flag_(false),
      landing_mode_flag_(false),
      landed_flag_(false),
      un_descend_flag_(false),
      landing_height_(0)
  {
    /* ros param */
    nhp_.param ("param_verbose", param_verbose_, true);

    nhp_.param ("estimate_mode", estimate_mode_, 0); //EGOMOTION_ESTIMATE: 0
    ROS_WARN("estimate_mode is %s", (estimate_mode_ == EGOMOTION_ESTIMATE)?string("EGOMOTION_ESTIMATE").c_str():((estimate_mode_ == EXPERIMENT_ESTIMATE)?string("EXPERIMENT_ESTIMATE").c_str():((estimate_mode_ == GROUND_TRUTH)?string("GROUND_TRUTH").c_str():string("WRONG_MODE").c_str())));

    odom_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/uav/state", 1);
    full_state_pub_ = nh_.advertise<aerial_robot_base::States>("/uav/full_state", 1);
    fuser_[0].resize(0);
    fuser_[1].resize(0);


    for(int i = 0; i < STATE_NUM; i ++)
      {
        state_[i].first = RAW;
        for(int j = 0; j < 3; j++) (state_[i].second)[j] = tf::Vector3(0, 0, 0);
      }

    /* TODO: represented sensors unhealth level */
    unhealth_level_ = 0;
  }

  virtual ~BasicEstimator(){}

  //mode
  static constexpr int RAW = -1;
  static constexpr int EGOMOTION_ESTIMATE = 0;
  static constexpr int EXPERIMENT_ESTIMATE = 1;
  static constexpr int GROUND_TRUTH = 2;

  static constexpr uint8_t STATE_NUM = 11;
  static constexpr uint8_t X_W = 0; //x in world coord
  static constexpr uint8_t Y_W = 1; //y in world coord
  static constexpr uint8_t Z_W = 2; //z in world coord
  static constexpr uint8_t ROLL_W = 3; //roll of cog in world coord
  static constexpr uint8_t PITCH_W = 4; //pitch of cog in world coord
  static constexpr uint8_t YAW_W = 5; //yaw of cog in world coord
  static constexpr uint8_t X_B = 6; //x in board coord
  static constexpr uint8_t Y_B = 7; //y in board coord
  static constexpr uint8_t ROLL_W_B = 8; //roll of mcu board in world coord
  static constexpr uint8_t PITCH_W_B = 9; //pitch of mcu board in world coord
  static constexpr uint8_t YAW_W_B = 10; //yaw of mcu board in world coord

   int getStateStatus( uint8_t axis)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    assert(axis < STATE_NUM);
    return state_[axis].first;
  }

  void setStateStatus( uint8_t axis,  int status)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    assert(axis < STATE_NUM);
    state_[axis].first = status;
  }

  /* axis: state axis (11) */
   State3Mode getState( uint8_t axis)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    assert(axis < STATE_NUM);
    return state_[axis].second;
  }
  /*
    axis: state axis (11)
    estimate_mode: egomotion/experiment/ground_truth
  */
   tf::Vector3 getState( uint8_t axis,  uint8_t estimate_mode)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);

    assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);

    return state_[axis].second[estimate_mode];
  }
  void setState( uint8_t axis,  int estimate_mode,  tf::Vector3 state)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);

    assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);

    state_[axis].second[estimate_mode] = state;
  }

  void setState( uint8_t axis,  int estimate_mode,  uint8_t state_mode,  float value)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);

    assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);
    assert(state_mode <= GROUND_TRUTH && state_mode >= EGOMOTION_ESTIMATE);

    (state_[axis].second[estimate_mode])[state_mode] = value;
  }


  inline void setSensorFusionFlag(bool flag){sensor_fusion_flag_ = flag;  }
  inline bool getSensorFusionFlag(){return sensor_fusion_flag_; }

  //start flying flag (~takeoff)
  virtual bool getFlyingFlag() {  return  flying_flag_;}
  virtual void setFlyingFlag(bool flag){  flying_flag_ = flag;}
  //start landing mode
  virtual bool getLandingMode() {  return  landing_mode_flag_;}
  virtual void setLandingMode(bool flag){  landing_mode_flag_ = flag;}
  // landed flag (acc_z check, ground shock)
  virtual bool getLandedFlag() {  return  landed_flag_;}
  virtual void setLandedFlag(bool flag){  landed_flag_ = flag;}
  /* when takeoff, should use the undescend mode be true */
  inline void setUnDescendMode(bool flag){un_descend_flag_ = flag;  }
  inline bool getUnDescendMode(){return un_descend_flag_; }
  /* landing height is set for landing to different terrain */
  virtual void setLandingHeight(float landing_height){ landing_height_ = landing_height;}
  virtual float getLandingHeight(){ return landing_height_;}

  const SensorFuser& getFuser(int mode)
  {
    assert(mode >= 0 && mode < 2);
    return fuser_[mode];

  }

  inline int getEstimateMode() {return estimate_mode_;}
  inline void setEstimateMode(int estimate_mode) {estimate_mode_ = estimate_mode;}

  /* sensor unhealth level */
  static constexpr uint8_t UNHEALTH_LEVEL1 = 1; // do nothing
  static constexpr uint8_t UNHEALTH_LEVEL2 = 2; // change estimation mode
  static constexpr uint8_t UNHEALTH_LEVEL3 = 3; // force landing


  /* set unhealth level */
  void setUnhealthLevel(uint8_t unhealth_level)
  {
    if(unhealth_level > unhealth_level_)
      unhealth_level_ = unhealth_level;

    /* TODO: should write the solution for the unhealth sensor  */
  }

  inline uint8_t getUnhealthLevel() { return unhealth_level_; }

protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher full_state_pub_, odom_state_pub_;
  ros::Subscriber estimate_height_mode_sub_;

  boost::mutex state_mutex_;   /* mutex */
  /* ros param */
  bool param_verbose_;

  int estimate_mode_; /* main estimte mode */

  /* 9: x_w, y_w, z_w, roll_w, pitch_w, yaw_cog_w, x_b, y_b, yaw_board_w */
  array<StateWithStatus, 11> state_;

  /* sensor fusion */
  boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> > sensor_fusion_loader_ptr_;
  bool sensor_fusion_flag_;
  array<SensorFuser, 2> fuser_; //0: egomotion; 1: experiment

  /* sensor (un)health level */
  uint8_t unhealth_level_;

  /* height related var */
  bool flying_flag_;
  bool landing_mode_flag_;
  bool landed_flag_;
  bool un_descend_flag_;
  float landing_height_;
};


#endif
