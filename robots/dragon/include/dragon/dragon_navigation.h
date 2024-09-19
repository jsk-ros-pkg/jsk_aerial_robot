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

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <sensor_msgs/JointState.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_navigation
{
  class DragonNavigator : public BaseNavigator
  {
  public:
    DragonNavigator();
    ~DragonNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

    inline const bool getLandingFlag() const { return landing_flag_; }

  private:
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Publisher joint_control_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;
    ros::Subscriber target_baselink_rot_sub_;

    void halt() override;
    void reset() override;

    void servoTorqueProcess();
    void landingProcess();
    void gimbalControl();
    void baselinkRotationProcess();
    void rosParamInit() override;

    void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void targetBaselinkRotCallback(const spinal::DesireCoordConstPtr& msg);

    /* target baselink rotation */
    double prev_rotation_stamp_;
    std::vector<double> target_gimbal_angles_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* landing process */
    bool level_flag_;
    bool landing_flag_;
    bool servo_torque_;

    /* rosparam */
    double height_thresh_;
    string joints_torque_control_srv_name_, gimbals_torque_control_srv_name_;
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};
