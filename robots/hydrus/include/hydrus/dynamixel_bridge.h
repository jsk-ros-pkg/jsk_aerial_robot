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


#ifndef HYDRUS_DYNAMIXEL_BRIDGE_H
#define HYDRUS_DYNAMIXEL_BRIDGE_H

// **** the dynamicxel motor number is opposite to the joint num, should be fixed

/* ros */
#include <ros/ros.h>

/* ros msg */
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <hydrus/ServoStates.h>
#include <hydrus/ServoControlCmd.h>
#include <hydrus/ServoTorqueCmd.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <dynamixel_msgs/MotorStateList.h>

/* util */
#include <string>

using namespace std;

class JointHandle
{
public:
  JointHandle(ros::NodeHandle nh, ros::NodeHandle nhp, int id):
    nh_(nh), nhp_(nhp),
    id_(id), current_val_(0), prev_val_(0), target_val_(0),
    sgn_(0), offset_(0), scale_(0), max_(0), min_(0),
    moving_(0), load_(0), temp_(0), error_(0)
  {
    std::stringstream id_str;
    id_str << id_ + 1;

    string joint_id =  std::string("joint") + id_str.str();

    nhp_.param(joint_id + std::string("_angle_max"), max_, 1.57); //real angle
    nhp_.param(joint_id + std::string("_angle_min"), min_, -1.57); //real angle
    nhp_.param(joint_id + std::string("_angle_sgn"), sgn_, 1);
    nhp_.param(joint_id + std::string("_angle_offset"), offset_, 0.0);
    nhp_.param(joint_id + std::string("_angle_scale"), scale_, 1.0);
    nhp_.param(joint_id + std::string("_name"), name_, joint_id);

    ROS_INFO("%s attribute: angle_max: %f, angle_min: %f, angle_scale: %f, angle_sng: %d, angle_offset: %f",
             name_.c_str(), max_, min_, scale_, sgn_, offset_);

    /* special pub/sub for ros::dynamixel system */
    string topic_name;
    topic_name = std::string("/j") + id_str.str()  + std::string("_controller/command");
    joint_ctrl_pub_ = nh_.advertise<std_msgs::Float64>(topic_name, 1);

    topic_name = std::string("/j") + id_str.str()  + std::string("_controller/state");
    joint_state_sub_ = nh_.subscribe(topic_name, 1, &JointHandle::jointCallback, this);

  };
  ~JointHandle() {};

  boost::shared_ptr<JointHandle> getHandle() { return boost::shared_ptr<JointHandle>(this); }

  inline void setCurrentVal(const double& val) { current_val_ = scale_ * sgn_ * (val - offset_); } //scaling
  inline void setPrevVal(const double& val){ prev_val_ = val; }
  inline void setTargetVal(const double& val){ target_val_ = val; }
  inline void setName(const string& name){ name_ = name; }
  inline void setId(const int& id){ id_ = id; }
  inline void setSgn(const int& sgn){ sgn_ = sgn; }
  inline void setOffset(const double& offset){ offset_ = offset; }
  inline void setScale(const double& scale){ scale_ = scale; }
  inline void setMax(const double& max){ max_ = max; }
  inline void setMin(const double& min){ min_ = min; }
  inline void setMoving(const bool& moving){ moving_ = moving; }
  inline void setLoad(const int16_t& load){ load_ = load; }
  inline void setTemp(const uint8_t& temp){ temp_ = temp; }
  inline void setError(const uint8_t& error){ error_ = error; }

  inline double getCurrentVal(){return current_val_;}
  inline double getPrevVal(){return prev_val_; }
  inline double getTargetVal(){return target_val_ * sgn_ / scale_ + offset_; }  //scaling
  inline string getName(){return name_; }
  inline int getId(){return id_; }
  inline int getSgn(){return sgn_; }
  inline double getOffget(){return offset_; }
  inline double getScale(){return scale_; }
  inline double getMax(){return max_; }
  inline double getMin(){return min_; }
  inline bool getMoving(){return moving_; }
  inline int16_t getLoad(){return load_; }
  inline uint8_t getTemp(){return temp_; }
  inline uint8_t getError(){return error_; }

  void pubTarget()
  {
    std_msgs::Float64 command;
    command.data = getTargetVal();
    joint_ctrl_pub_.publish(command);
  }

  bool TorqueControl(dynamixel_controllers::TorqueEnable::Request &req)
  {
    std::stringstream joint_no;
    joint_no << id_ + 1;

    std::string srv_name = std::string("/j") + joint_no.str()  + std::string("_controller/torque_enable");

    ros::ServiceClient client = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(srv_name);
    dynamixel_controllers::TorqueEnable srv;
    srv.request.torque_enable = req.torque_enable;
    if (client.call(srv))
      {
        if(req.torque_enable) ROS_INFO("%s: enable torque", name_.c_str());
        else ROS_INFO("%s: disable torque", name_.c_str());
        return true;
      }
    else
      {
        return false;
      }

  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_ctrl_pub_;

  int id_;
  string name_;
  double current_val_;
  double prev_val_; /* the previous servo angle in 14bit */
  double target_val_;
  //uint16_t target_servo_angle_; // same with "target_joint_angle", but different resolution.
  int sgn_;
  double offset_;
  double scale_;
  double max_;
  double min_;
  bool moving_; /* for dynamixel_msgs */
  int16_t load_; /* for dynamixel_msgs, if 8bit, it means load, if 16bit, it means current  */
  uint8_t temp_; /* for dynamixel_msgs */
  uint8_t error_; /* for dynamixel_msgs */

  void jointCallback(const dynamixel_msgs::JointStateConstPtr& msg)
  {
    setCurrentVal(msg->current_pos);
  }

};

typedef boost::shared_ptr<JointHandle> JointHandlePtr;

namespace hydrus
{
  class JointInterface
  {
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    vector<JointHandlePtr> joints_;

    uint16_t servo_on_mask_, servo_full_on_mask_;
    double moving_check_rate_;
    double moving_angle_thresh_;
    bool overload_check_;  /* check overload automatically */
    bool start_joint_control_;
    bool send_init_joint_pose_;
    int joint_num_;
    int  bridge_mode_;
    int send_init_joint_pose_cnt_;

    double bridge_rate_;
    ros::Timer bridge_timer_;

    ros::Subscriber servo_angle_sub_; //current servo angles from MCU
    ros::Subscriber joints_ctrl_sub_;
    ros::Publisher servo_ctrl_pub_; //target servo angles to MCU
    ros::Publisher servo_torque_cmd_pub_; //torque enable/disable to MCU
    ros::Publisher joints_state_pub_;
    ros::Publisher dynamixel_msg_pub_;

    ros::ServiceServer joints_torque_control_srv_;
    ros::ServiceServer overload_check_activate_srv_;

    virtual void servoStatesCallback(const hydrus::ServoStatesConstPtr& state_msg);
    virtual void jointsCtrlCallback(const sensor_msgs::JointStateConstPtr& joints_ctrl_msg);
    virtual bool jointsTorqueEnableCallback(dynamixel_controllers::TorqueEnable::Request &req, dynamixel_controllers::TorqueEnable::Response &res);
    virtual void bridgeFunc(const ros::TimerEvent & e);

    virtual bool overloadCheckActivateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
      overload_check_ = req.data;

      res.success = true;
      return true;
    }

  public:
    static const uint8_t DYNAMIXEL_HUB_MODE = 0;
    static const uint8_t MCU_MODE = 1;
    static const uint8_t OVERLOAD_FLAG = 0x20;

    JointInterface(ros::NodeHandle nh, ros::NodeHandle nhp);
    virtual ~JointInterface()  {}
    virtual void jointStatePublish();

  };
};

#endif
