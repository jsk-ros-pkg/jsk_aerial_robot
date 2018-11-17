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


/* util */
#include <string>
#include <boost/algorithm/clamp.hpp>

using namespace std;

namespace ValueType
{
  enum
    {BIT, RADIAN};
};

class SingleServoHandle
{
public:
  SingleServoHandle(string name, int id, int sgn, double offset, double scale, bool receive_real_state):
    name_(name), id_(id), curr_val_(0), target_val_(0),init_target_val_(false), sgn_(sgn),
    offset_(offset), scale_(scale), receive_real_state_(receive_real_state)
  {
    /* for simulation */
    //joint_ctrl_pub_ = nh_.advertise<std_msgs::Float64>(std::string("/j") + std_  + std::string("_controller/command"), 1);

  };
  ~SingleServoHandle() {};

  boost::shared_ptr<SingleServoHandle> getHandle() { return boost::shared_ptr<SingleServoHandle>(this); }

  inline void setCurrVal(const double& val, int value_type)
  {
    if(value_type == ValueType::BIT)
      {
        if (val < 0)
          {
            ROS_ERROR("%s: bit current val could not be negative: %f", name_.c_str(), val);
            return;
          }

        curr_val_ = scale_ * sgn_ * (val - offset_);
      }
    else if(value_type == ValueType::RADIAN)
      curr_val_ = val;
    else
      ROS_ERROR("wrong value type");

    if(!init_target_val_)
      {
        target_val_ = val;
        init_target_val_ = true;
      }
  }

  inline void setTargetVal(const double& val, int value_type)
  {
    if(value_type == ValueType::BIT)
      {
        if (val < 0)
          {
            ROS_ERROR("%s: bit target val could not be negative: %f", name_.c_str(), val);
            return;
          }
        target_val_ = scale_ * sgn_ * (val - offset_);
      }
    else if(value_type == ValueType::RADIAN)
      target_val_ = val;
    else
      ROS_ERROR("%s: wrong value type", name_.c_str());

    if(!receive_real_state_) curr_val_ = target_val_;
  }

  inline void setName(const string& name){ name_ = name; }
  inline void setId(const int& id){ id_ = id; }
  inline void setSgn(const int& sgn){ sgn_ = sgn; }
  inline void setOffset(const int& offset){ offset_ = offset; }
  inline void setScale(const double& scale){ scale_ = scale; }

  const double getCurrVal(int value_type) const
  {
    if(value_type == ValueType::BIT)
      {
        return boost::algorithm::clamp(curr_val_ * sgn_ / scale_ + offset_, 0, UINT16_MAX);
      }
    else if(value_type == ValueType::RADIAN)
      return curr_val_;
  }

  const double getTargetVal(int value_type) const
  {
    if(value_type == ValueType::BIT)
      {
        return boost::algorithm::clamp(target_val_ * sgn_ / scale_ + offset_, 0, UINT16_MAX);
      }
    else if(value_type == ValueType::RADIAN)
      return target_val_;
  }

  inline const string& getName(){return name_; }
  inline const int& getId() const {return id_; }
  inline const int& getSgn() const {return sgn_; }
  inline const int& getOffset() const {return offset_; }
  inline const double& getScale() const {return scale_; }

private:
  int id_;
  string name_;
  double curr_val_; // radian
  double target_val_; // radian
  int sgn_;
  int offset_;
  double scale_;

  bool receive_real_state_;
  bool init_target_val_;
};

using SingleServoHandlePtr= boost::shared_ptr<SingleServoHandle>;
using ServoGroupHandler= vector<SingleServoHandlePtr>;

class ServoBridge
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher servo_states_pub_;
  vector<ros::Subscriber> servo_states_subs_;
  vector<ros::Subscriber> servo_ctrl_subs_;
  map<string, ros::Publisher> servo_ctrl_pubs_;
  vector<ros::ServiceServer> servo_torque_ctrl_srvs_;
  map<string, ros::Publisher> servo_torque_ctrl_pubs_;

  map<string, ServoGroupHandler> servos_handler_;
  double moving_check_rate_;
  double moving_angle_thresh_;
  bool send_init_joint_pose_;
  int send_init_joint_pose_cnt_;


  void servoStatesCallback(const spinal::ServoStatesConstPtr& state_msg, const std::string& servo_group_name);
  void servoCtrlCallback(const sensor_msgs::JointStateConstPtr& joints_ctrl_msg, const std::string& servo_group_name);
  bool servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name);

public:
  ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~ServoBridge()  {}
  void servoStatePublish();

};


#endif
