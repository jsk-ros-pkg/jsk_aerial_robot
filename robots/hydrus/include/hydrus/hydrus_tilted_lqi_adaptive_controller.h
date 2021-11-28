// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#pragma once

#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <spinal/DesireYaw.h>
#include <spinal/SetMRACParams.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt8.h>

// flight state enum
#define ARM_OFF_STATE 0
#define START_STATE 1
#define ARM_ON_STATE 2
#define TAKEOFF_STATE 3
#define LAND_STATE 4
#define HOVER_STATE 5
#define STOP_STATE 6

namespace aerial_robot_control
{
  class HydrusTiltedLQIAdaptiveController: public HydrusTiltedLQIController
  {
  public:
    HydrusTiltedLQIAdaptiveController() {}
    virtual ~HydrusTiltedLQIAdaptiveController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:

    ros::Publisher desire_yaw_pub_;
    spinal::DesireYaw desire_yaw_msg_;

    int flight_state_;
    int prev_flight_state_;
    int takeoff_wait_time_;
    ros::Time takeoff_time_;
    bool is_using_mrac_;

    ros::Subscriber flight_state_sub_;

    void controlCore() override;
    void publishGain() override;

    void spinalMRACTrigger(bool trigger);

    void flightStateCallback(const std_msgs::UInt8::ConstPtr& msg);
  };
};
