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

#ifndef DRAGON_DYNAMIXEL_BRIDGE_H
#define DRAGON_DYNAMIXEL_BRIDGE_H

#include <hydrus/dynamixel_bridge.h>

namespace dragon
{
  class JointInterface: public hydrus::JointInterface
  {
  protected:
    ros::Subscriber gimbal_ctrl_sub_; //target gimbal angles from ros
    ros::Publisher gimbal_ctrl_pub_; //target gimbal angles to MCU
    ros::Publisher gimbal_config_cmd_pub_; //config command to MCU
    ros::ServiceServer gimbals_torque_control_srv_;

    vector<JointHandlePtr> gimbals_;
    int gimbal_num_;
    bool start_gimbal_control_;

    void gimbalsCtrlCallback(const sensor_msgs::JointStateConstPtr& gimbals_ctrl_msg);
    bool gimbalsTorqueEnableCallback(dynamixel_controllers::TorqueEnable::Request &req, dynamixel_controllers::TorqueEnable::Response &res);

    void bridgeFunc(const ros::TimerEvent & e);
  public:
    JointInterface(ros::NodeHandle nh, ros::NodeHandle nhp);

    ~JointInterface() {}

    void jointStatePublish();
  };
};

#endif
