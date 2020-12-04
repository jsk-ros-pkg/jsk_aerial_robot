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

#include <aerial_robot_control/control/control_base.h>
#include <aerial_robot_control/control/utils/pid.h>
#include <aerial_robot_control/PidControlConfig.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <aerial_robot_msgs/PoseControlPid.h>
#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>
#include <spinal/YawFromPC.h>

using PidControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::PidControlConfig>;

namespace aerial_robot_control
{
  enum
    {
      X, Y, Z, ROLL, PITCH, YAW,
    };

  class PoseLinearController: public ControlBase
  {
  public:
    PoseLinearController();
    virtual ~PoseLinearController() = default;
    void virtual initialize(ros::NodeHandle nh,
                            ros::NodeHandle nhp,
                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                            boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                            double ctrl_loop_du) override;

    virtual bool update() override;
    virtual void reset() override;

  protected:
    ros::Publisher pid_pub_;

    std::vector<PID> pid_controllers_;
    std::vector<boost::shared_ptr<PidControlDynamicConfig> > pid_reconf_servers_;
    aerial_robot_msgs::PoseControlPid pid_msg_;

    ros::Publisher yaw_from_pc_pub_;
    spinal::YawFromPC yaw_from_pc_msg_;

    bool need_yaw_d_control_;
    bool start_rp_integration_;
    double start_rp_integration_height_;

    double landing_err_z_;
    double safe_landing_height_;

    tf::Vector3 pos_, target_pos_;
    tf::Vector3 vel_, target_vel_;
    tf::Vector3 target_acc_;
    tf::Vector3 rpy_, target_rpy_;
    tf::Vector3 omega_, target_omega_;

    virtual void controlCore();
    virtual void sendCmd();


    void cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices);
  };

};
