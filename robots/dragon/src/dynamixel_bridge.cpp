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


#include <dragon/dynamixel_bridge.h>

using namespace std;

namespace dragon
{

  ServoInterface::ServoInterface(ros::NodeHandle nh, ros::NodeHandle nhp)
    : hydrus::JointInterface(nh, nhp), gimbals_(0)
  {
    nhp_.param("gimbal_num", gimbal_num_, 0);
    for(int i = 0; i < gimbal_num_; i++)
      {
        gimbals_.push_back(ServoHandlePtr(new ServoHandle(nh, ros::NodeHandle(nhp, "gimbal"), i, false)));
      }

    gimbal_ctrl_sub_ = nh_.subscribe("gimbals_ctrl", 1, &ServoInterface::gimbalsCtrlCallback, this) ;

    gimbals_torque_control_srv_ =  nh_.advertiseService("/gimbals/torque_enable", &ServoInterface::gimbalsTorqueEnableCallback, this);

  }

  void ServoInterface::gimbalsCtrlCallback(const sensor_msgs::JointStateConstPtr& gimbals_ctrl_msg)
  {
    if(gimbals_ctrl_msg->position.size() != gimbal_num_)
      {
        ROS_ERROR("[dynamixel bridge, gimbal control]: the gimbal num from rosparam %d is not equal with ros msgs %d", gimbal_num_, (int)gimbals_ctrl_msg->position.size());
        return;
      }

    hydrus::ServoControlCmd target_angle_msg;

    for(int i = 0; i < gimbal_num_; i ++)
      {
        gimbals_[i]->setTargetVal(gimbals_ctrl_msg->position[i]);
        gimbals_[i]->setConvertedCurrentVal(gimbals_ctrl_msg->position[i]); /* fedd-forward mode */
        target_angle_msg.index.push_back(gimbals_[i]->getId());
        target_angle_msg.angles.push_back(gimbals_[i]->getTargetVal());
      }

    servo_ctrl_pub_.publish(target_angle_msg);
  }

  bool ServoInterface::gimbalsTorqueEnableCallback(dynamixel_controllers::TorqueEnable::Request &req, dynamixel_controllers::TorqueEnable::Response &res)
  {
    /* direct send torque control flag */
    hydrus::ServoTorqueCmd torque_off_msg;
    for(auto it = gimbals_.begin(); it != gimbals_.end(); ++it)
      {
        torque_off_msg.index.push_back((*it)->getId());
        torque_off_msg.torque_enable.push_back(req.torque_enable);
      }
    servo_torque_cmd_pub_.publish(torque_off_msg);

    return true;
  }

  void ServoInterface::jointStatePublish()
  {
    sensor_msgs::JointState joints_state_msg;
    joints_state_msg.header.stamp = ros::Time::now();

    /* joints */
    for(auto it = joints_.begin(); it != joints_.end(); ++it)
      {
        joints_state_msg.name.push_back((*it)->getName());
        joints_state_msg.position.push_back((*it)->getCurrentVal());
      }

    /* gimbals */
    for(auto it = gimbals_.begin(); it != gimbals_.end(); ++it)
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
};

