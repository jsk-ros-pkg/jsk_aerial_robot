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


#include <aerial_robot_model/servo_bridge.h>

ServoBridge::ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh),nhp_(nhp)
{
  XmlRpc::XmlRpcValue all_servos_params;
  nh_.getParam("servo_controller", all_servos_params);

  for(auto servo_group_params: all_servos_params)
    {
      ROS_ASSERT(servo_group_params.second.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      ServoGroupHandler servo_group_handler;
      for(auto servo : servo_group_params.second)
        {
          if(servo.first.find("controller") != string::npos)
            {
              int sgn = (servo.second.hasMember("angle_sgn"))?
                servo.second["angle_sgn"]:servo_group_params.second["angle_sgn"];
              int offset = (servo.second.hasMember("zero_point_offset"))?
                servo.second["zero_point_offset"]:servo_group_params.second["zero_point_offset"];
              double scale = (servo.second.hasMember("angle_scale"))?
                servo.second["angle_scale"]:servo_group_params.second["angle_scale"];

              servo_group_handler.push_back(SingleServoHandlePtr(new SingleServoHandle(servo.second["name"], servo.second["id"], sgn, offset, scale, servo_group_params.second.hasMember("state_sub_topic"))));
            }
        }

      servos_handler_.insert(make_pair(servo_group_params.first, servo_group_handler));

      /* ros pub/sub, service */
      /* Get servo states (i.e. joint angles) from real machine, if necessary */
      if(servo_group_params.second.hasMember("state_sub_topic"))
        servo_states_subs_.push_back(nh_.subscribe<spinal::ServoStates>((string)servo_group_params.second["state_sub_topic"], 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, servo_group_params.first)));

      /* subscribe target servo state from controller */
      servo_ctrl_subs_.push_back(nh_.subscribe<sensor_msgs::JointState>(servo_group_params.first + string("_ctrl"), 10, boost::bind(&ServoBridge::servoCtrlCallback, this, _1, servo_group_params.first)));

      /* publish target servo state to real machine */
      servo_ctrl_pubs_.insert(make_pair(servo_group_params.first, nh_.advertise<spinal::ServoControlCmd>(servo_group_params.second["ctrl_pub_topic"], 1)));

      /* torque on/off */
      if(servo_group_params.second.hasMember("torque_pub_topic"))
        {
          servo_torque_ctrl_srvs_.push_back(nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(servo_group_params.first + string("/torque_enable"), boost::bind(&ServoBridge::servoTorqueCtrlCallback, this, _1, _2, servo_group_params.first)));
          servo_torque_ctrl_pubs_.insert(make_pair(servo_group_params.first, nh_.advertise<spinal::ServoTorqueCmd>((string)servo_group_params.second["torque_pub_topic"], 1)));
        }
    }
  servo_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

}

void ServoBridge::servoStatesCallback(const spinal::ServoStatesConstPtr& state_msg, const string& servo_group_name)
{
  if(state_msg->servos.size() != servos_handler_[servo_group_name].size())
    {
      ROS_ERROR("[servo bridge, servo state callback]: the joint num from rosparam %d is not equal with ros msgs %d", (int)servos_handler_[servo_group_name].size(), (int)state_msg->servos.size());
      return;
    }

  for(auto it: state_msg->servos)
    {
      auto servo_handler = find_if(servos_handler_[servo_group_name].begin(),
                                   servos_handler_[servo_group_name].end(),
                                   [&](SingleServoHandlePtr s) {return it.index == s->getId();} );

      if(servo_handler == servos_handler_[servo_group_name].end())
        {
          ROS_ERROR("[servo bridge, servo state callback]: no matching joint handler for servo index %d", it.index);
          return;
        }
      (*servo_handler)->setCurrVal((double)it.angle, ValueType::BIT);
    }

  sensor_msgs::JointState servo_states_msg;
  servo_states_msg.header.stamp = state_msg->stamp;

  for(auto servo_group : servos_handler_)
    {
      for(auto servo_handler: servo_group.second)
        {
          servo_states_msg.name.push_back(servo_handler->getName());
          servo_states_msg.position.push_back(servo_handler->getCurrVal(ValueType::RADIAN));
        }
    }
  servo_states_pub_.publish(servo_states_msg);
}

void ServoBridge::servoCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  spinal::ServoControlCmd target_angle_msg;

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */

          if(servo_ctrl_msg->position.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo control control]: the servo position num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->position.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            return;
          }

          (*servo_handler)->setTargetVal(servo_ctrl_msg->position[i], ValueType::RADIAN);
          target_angle_msg.index.push_back((*servo_handler)->getId());
          target_angle_msg.angles.push_back((*servo_handler)->getTargetVal(ValueType::BIT));
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->position.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo control control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->position.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->position.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          servo_handler->setTargetVal(servo_ctrl_msg->position[i], ValueType::RADIAN);
          target_angle_msg.index.push_back(servo_handler->getId());
          target_angle_msg.angles.push_back(servo_handler->getTargetVal(ValueType::BIT));
        }
    }

  servo_ctrl_pubs_[servo_group_name].publish(target_angle_msg);
}

bool ServoBridge::servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name)
{
  spinal::ServoTorqueCmd torque_off_msg;
  for(auto servo_handler: servos_handler_[servo_group_name])
    {
      torque_off_msg.index.push_back(servo_handler->getId());
      torque_off_msg.torque_enable.push_back(req.data);
    }

  servo_torque_ctrl_pubs_[servo_group_name].publish(torque_off_msg);

  return true;
}

