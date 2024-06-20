// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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


#ifndef SERVO_BRIDGE_H
#define SERVO_BRIDGE_H

/* ros */
#include <ros/ros.h>

/* ros msg */
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/JointProfiles.h>
#include <spinal/UavInfo.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <urdf/model.h>

/* filter */
#include <kalman_filter/lpf_filter.h>

/* util */
#include <string>
#include <boost/algorithm/clamp.hpp>

using namespace std;

namespace ValueType
{
  enum
    {BIT = 0, RADIAN = 1};
};

class SingleServoHandle
{
public:
  SingleServoHandle(string name, int id, int angle_sgn, double zero_point_offset, double angle_scale, double upper_limit, double lower_limit, double torque_scale, bool receive_real_state, bool filter_flag = false, double sample_freq = 0, double cutoff_freq = 0): name_(name), id_(id), curr_angle_val_(0), target_angle_val_(0), init_target_angle_val_(false), curr_torque_val_(0), angle_sgn_(angle_sgn), zero_point_offset_(zero_point_offset), angle_scale_(angle_scale), upper_limit_(upper_limit), lower_limit_(lower_limit), torque_scale_(torque_scale), receive_real_state_(receive_real_state), filter_flag_(filter_flag)
  {
    /* for simulation */
    //joint_ctrl_pub_ = nh_.advertise<std_msgs::Float64>(std::string("/j") + std_  + std::string("_controller/command"), 1);

    if(filter_flag_)
      {
        if(sample_freq == 0 || cutoff_freq == 0)
          throw std::runtime_error("filtering config for is invalid");

        lpf_angle_ = IirFilter(sample_freq, cutoff_freq, 1);
      }
  };
  ~SingleServoHandle() {};

  boost::shared_ptr<SingleServoHandle> getHandle() { return boost::shared_ptr<SingleServoHandle>(this); }

  inline void setCurrAngleVal(const double& val, int value_type)
  {
    if(value_type == ValueType::BIT)
      curr_angle_val_ = angle_scale_ * angle_sgn_ * (val - zero_point_offset_);
    else if(value_type == ValueType::RADIAN)
      curr_angle_val_ = val;
    else
      ROS_ERROR("wrong value type");

    /* do low pass filtering */
    if(filter_flag_)
      curr_angle_val_ = lpf_angle_.filterFunction(curr_angle_val_);

    if(!init_target_angle_val_)
      {
        target_angle_val_ = boost::algorithm::clamp(val, lower_limit_, upper_limit_);
        init_target_angle_val_ = true;
      }
  }

  inline void setTargetAngleVal(const double& val, int value_type)
  {
    if(value_type == ValueType::BIT)
      target_angle_val_ = boost::algorithm::clamp(angle_scale_ * angle_sgn_ * (val - zero_point_offset_), lower_limit_, upper_limit_);
    else if(value_type == ValueType::RADIAN)
      target_angle_val_ = boost::algorithm::clamp(val, lower_limit_, upper_limit_);
    else
      ROS_ERROR("%s: wrong value type", name_.c_str());

    if(!receive_real_state_) curr_angle_val_ = target_angle_val_;
  }

  inline void setCurrTorqueVal(const double& val)
  {
      curr_torque_val_ = torque_scale_ * angle_sgn_ * val;
  }

  inline void setName(const string& name){ name_ = name; }
  inline void setId(const int& id){ id_ = id; }
  inline void setAngleSgn(const int& sgn){ angle_sgn_ = sgn; }
  inline void setZeroPointOffset(const int& offset){ zero_point_offset_ = offset; }
  inline void setAngleScale(const double& scale){ angle_scale_ = scale; }
  inline void setTorqueScale(const double& scale){ torque_scale_ = scale; }

  const double getCurrAngleVal(int value_type) const
  {
    if(value_type == ValueType::BIT)
      {
        return boost::algorithm::clamp(curr_angle_val_ * angle_sgn_ / angle_scale_ + zero_point_offset_, INT16_MIN/2, INT16_MAX/2);
      }
    else // ValueType::RADIAN
      {
        return curr_angle_val_;
      }
  }

  const double getTargetAngleVal(int value_type) const
  {
    if(value_type == ValueType::BIT)
      {
        return boost::algorithm::clamp(target_angle_val_ * angle_sgn_ / angle_scale_ + zero_point_offset_, INT16_MIN/2, INT16_MAX/2);
      }
    else // ValueType::RADIAN
      {
        return target_angle_val_;
      }
  }

  inline const double getCurrTorqueVal() const
  {
    return curr_torque_val_;
  }

  inline const string& getName(){return name_; }
  inline const int& getId() const {return id_; }
  inline const int& getAngleSgn() const {return angle_sgn_; }
  inline const int& getZeroPointOffset() const {return zero_point_offset_; }
  inline const double& getAngleScale() const {return angle_scale_; }
  inline const double& getTorqueScale() const {return torque_scale_; }

private:
  int id_;
  string name_;
  double curr_angle_val_; // radian
  double target_angle_val_; // radian
  double curr_torque_val_; // Nm
  int angle_sgn_;
  int zero_point_offset_;
  double angle_scale_;
  double lower_limit_, upper_limit_;
  double torque_scale_;

  bool receive_real_state_;
  bool filter_flag_;
  bool init_target_angle_val_;


  IirFilter lpf_angle_;
};

using SingleServoHandlePtr= boost::shared_ptr<SingleServoHandle>;
using ServoGroupHandler= vector<SingleServoHandlePtr>;

class ServoBridge
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher servo_states_pub_;
  ros::Publisher mujoco_control_input_pub_;
  ros::Publisher joint_profile_pub_;
  ros::Subscriber uav_info_sub_;
  map<string, ros::Subscriber> servo_states_subs_;
  map<string, ros::Subscriber> servo_ctrl_subs_;
  map<string, bool> no_real_state_flags_;
  map<string, ros::Publisher> servo_ctrl_pubs_;
  map<string, ros::ServiceServer> servo_torque_ctrl_srvs_;
  map<string, ros::Publisher> servo_torque_ctrl_pubs_;
  map<string, vector<ros::Publisher> > servo_ctrl_sim_pubs_; // TODO: should be actionlib, trajectory controller

  map<string, ServoGroupHandler> servos_handler_;
  double moving_check_rate_;
  double moving_angle_thresh_;
  bool send_init_joint_pose_;
  bool simulation_mode_;
  bool use_mujoco_;
  int send_init_joint_pose_cnt_;

  void servoStatesCallback(const spinal::ServoStatesConstPtr& state_msg, const std::string& servo_group_name);
  void servoCtrlCallback(const sensor_msgs::JointStateConstPtr& joints_ctrl_msg, const std::string& servo_group_name);
  bool servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name);
  void uavInfoCallback(const spinal::UavInfoConstPtr& uav_msg);

public:
  ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~ServoBridge()  {}
  void servoStatePublish();

};


#endif
