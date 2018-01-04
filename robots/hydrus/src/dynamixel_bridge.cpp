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


#include <hydrus/dynamixel_bridge.h>

using namespace std;

namespace hydrus
{

  JointInterface::JointInterface(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh),nhp_(nhp),
      joints_(0), servo_on_mask_(0), servo_full_on_mask_(0),
      start_joint_control_(false), send_init_joint_pose_(false)
  {
    nhp_.param("joint_num", joint_num_, 3);
    nhp_.param("bridge_mode", bridge_mode_, 0);

    for(int i = 0; i < joint_num_; i++)
      {
        servo_full_on_mask_ |= (1 << i);
        joints_.push_back(ServoHandlePtr(new ServoHandle(nh, ros::NodeHandle(nhp, "joint"), i)));
      }

    joints_torque_control_srv_ =  nh_.advertiseService("/joints/torque_enable", &JointInterface::jointsTorqueEnableCallback, this);
    joints_ctrl_sub_ = nh_.subscribe("joints_ctrl", 1, &JointInterface::jointsCtrlCallback, this, ros::TransportHints().tcpNoDelay());
    joints_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    if(bridge_mode_ == MCU_MODE)
      {
        nhp_.param("moving_check_rate", moving_check_rate_, 10.0);
        nhp_.param("moving_angle_thresh", moving_angle_thresh_, 0.005); //[rad]

        string topic_name;
        nhp_.param("servo_sub_name", topic_name, std::string("/servo/states"));
        servo_angle_sub_ = nh_.subscribe(topic_name, 1, &JointInterface::jointStatesCallback, this, ros::TransportHints().tcpNoDelay());
        nhp_.param("servo_pub_name", topic_name, std::string("/servo/target_states"));
        servo_ctrl_pub_ = nh_.advertise<hydrus::ServoControlCmd>(topic_name, 1);
        nhp_.param("servo_torque_cmd_pub_name", topic_name, std::string("/servo/torque_enable"));
        servo_torque_cmd_pub_ = nh_.advertise<hydrus::ServoTorqueCmd>(topic_name, 1);

        /* overload check activate flag */
        nhp_.param("overload_check_activate_srv_name", topic_name, std::string("/overload_check_activate"));
        overload_check_activate_srv_ = nh_.advertiseService(topic_name, &JointInterface::overloadCheckActivateCallback, this);

        /* dynamixel msg */
        nhp_.param("dynamixel_msg_pub_name", topic_name, std::string("/motor_states/joints_port"));
        dynamixel_msg_pub_ = nh_.advertise<dynamixel_msgs::MotorStateList>(topic_name, 1);

      }

    nhp_.param("bridge_rate", bridge_rate_, 40.0);
    send_init_joint_pose_cnt_ = bridge_rate_;
    bridge_timer_ = nhp_.createTimer(ros::Duration(1.0 / bridge_rate_), &JointInterface::bridgeFunc, this);
  }

  void JointInterface::jointStatesCallback(const hydrus::ServoStatesConstPtr& state_msg)
  {
    /* the joint_num_ should be equal with the mcu information */
    if(state_msg->servos.size() != joints_.size())
      {
        ROS_ERROR("[dynamixel bridge, joint state]: the joint num from rosparam %d is not equal with ros msgs %d", (int)joints_.size(), (int)state_msg->servos.size());
        return;
      }

    static ros::Time prev_time = ros::Time::now();
    for(auto it = state_msg->servos.begin(); it != state_msg->servos.end(); ++it)
      {
        auto joint_handler = find_if(joints_.begin(), joints_.end(), [&](ServoHandlePtr s) {return it->index == s->getId();} );
        if(joint_handler == joints_.end())
          {
            ROS_ERROR("[dynamixel bridge]: no matching joint handler for servo index %d", it->index);
            continue;
          }

        (*joint_handler)->setCurrentVal(it->angle);
        (*joint_handler)->setError(it->error);
        (*joint_handler)->setTemp(it->temp);
        (*joint_handler)->setLoad(it->load);

        if(!(servo_on_mask_ & (1 << (distance(joints_.begin(), joint_handler)))))
          {
            ROS_WARN("[dynamixel bridge] set the initial target angle to current angle");
            (*joint_handler)->setTargetVal((*joint_handler)->getCurrentVal());
          }
        servo_on_mask_ |= (1 << (distance(joints_.begin(), joint_handler)));
      }

    /* check moving */
    if(ros::Time::now().toSec() - prev_time.toSec() > (1 / moving_check_rate_))
      {
        for(auto it = joints_.begin(); it != joints_.end(); ++it)
          {
            if(abs((*it)->getCurrentVal() - (*it)->getPrevVal()) > moving_angle_thresh_)
              {
                (*it)->setMoving(true);
                (*it)->setPrevVal((*it)->getCurrentVal());
              }
            else (*it)->setMoving(false);
          }
        prev_time = ros::Time::now();
      }

    /* check223 overload */
    if(overload_check_)
      {
        for(auto it = state_msg->servos.begin(); it != state_msg->servos.end(); ++it)
          {
            if(OVERLOAD_FLAG & it->error)
              {
                ROS_WARN("[dynamixel bridge]: motor: %d, overload", (int)distance(state_msg->servos.begin(), it) + 1);
                /* direct send torque control flag */
                hydrus::ServoTorqueCmd torque_off_msg;
                torque_off_msg.index.push_back(it->index);
                torque_off_msg.torque_enable.push_back(false);
                servo_torque_cmd_pub_.publish(torque_off_msg);
              }
          }
      }
  }

  void JointInterface::jointsCtrlCallback(const sensor_msgs::JointStateConstPtr& joints_ctrl_msg)
  {
    /* the joint control size should be equal with joint_num_ */
    if(joints_ctrl_msg->position.size() != joints_.size())
      {
        ROS_ERROR("[dynamixel bridge, joint control]: the joint num from rosparam %d is not equal with ros msgs %d", (int)joints_.size(), (int)joints_ctrl_msg->position.size());
        return;
      }

    hydrus::ServoControlCmd target_angle_msg;

    auto joint_handler = joints_.begin();
    for(int i = 0; i < joint_num_; i ++)
      {
        (*joint_handler)->setTargetVal(joints_ctrl_msg->position[i]);
        if(bridge_mode_ == DYNAMIXEL_HUB_MODE)
          (*joint_handler)->pubTarget();
        else if(bridge_mode_ == MCU_MODE)
          {
            target_angle_msg.index.push_back((*joint_handler)->getId());
            target_angle_msg.angles.push_back((*joint_handler)->getTargetVal());
          }
        joint_handler++;
      }

    if(bridge_mode_ == MCU_MODE) servo_ctrl_pub_.publish(target_angle_msg);
  }

  bool JointInterface::jointsTorqueEnableCallback(dynamixel_controllers::TorqueEnable::Request &req, dynamixel_controllers::TorqueEnable::Response &res)
  {
    if(bridge_mode_ == DYNAMIXEL_HUB_MODE)
      {
        for(auto it = joints_.begin(); it != joints_.end(); ++it) (*it)->TorqueControl(req);
      }
    else if(bridge_mode_ == MCU_MODE)
      {
        hydrus::ServoTorqueCmd torque_off_msg;
        for(auto it = joints_.begin(); it != joints_.end(); ++it)
          {
            torque_off_msg.index.push_back((*it)->getId());
            torque_off_msg.torque_enable.push_back(req.torque_enable);
          }
        servo_torque_cmd_pub_.publish(torque_off_msg);
      }
    return true;
  }

  void JointInterface::jointStatePublish()
  {
    sensor_msgs::JointState joints_state_msg;
    joints_state_msg.header.stamp = ros::Time::now();

    for(auto it = joints_.begin(); it != joints_.end(); ++it)
      {
        joints_state_msg.name.push_back((*it)->getName());
        joints_state_msg.position.push_back((*it)->getCurrentVal());
      }

    joints_state_pub_.publish(joints_state_msg);

    /* need to publish dynamixel msg */
    if(bridge_mode_ == MCU_MODE)
      {
        dynamixel_msgs::MotorStateList dynamixel_msg;
        for(auto it = joints_.begin(); it != joints_.end(); ++it)
          {
            dynamixel_msgs::MotorState motor_msg;
            motor_msg.timestamp = ros::Time::now().toSec();
            motor_msg.id = (*it)->getId();
            motor_msg.error = (*it)->getError();
            motor_msg.load = (*it)->getLoad();
            motor_msg.moving = (*it)->getMoving();
            motor_msg.temperature = (*it)->getTemp();
            dynamixel_msg.motor_states.push_back(motor_msg);
          }
        dynamixel_msg_pub_.publish(dynamixel_msg);
      }
  }

  void JointInterface::bridgeFunc(const ros::TimerEvent & e)
  {
    if(servo_on_mask_ != servo_full_on_mask_) return;

    jointStatePublish();
  }

};

