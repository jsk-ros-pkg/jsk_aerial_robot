// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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

#include <aerial_robot_control/control/pose_linear_controller.h>
#include <tiger/model/full_vectoring_robot_model.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float32MultiArray.h>

namespace aerial_robot_control
{
  namespace Tiger
  {
    class WalkController: public PoseLinearController
    {
    public:
      WalkController();
      ~WalkController() {}

      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                      double ctrl_loop_rate) override;
      bool update() override;

    private:

      ros::Publisher flight_cmd_pub_; //for spinal
      ros::Publisher gimbal_control_pub_;
      ros::Publisher joint_control_pub_;
      ros::Publisher target_vectoring_force_pub_;

      boost::shared_ptr<::Tiger::FullVectoringRobotModel> tiger_robot_model_;

      std::vector<float> target_base_thrust_;
      std::vector<double> target_gimbal_angles_;
      Eigen::VectorXd target_vectoring_f_;

      sensor_msgs::JointState target_joint_state_;
      sensor_msgs::JointState compliance_joint_state_;

      double joint_ctrl_rate_;
      double tor_kp_;

      void rosParamInit();
      virtual void sendCmd() override;
    };
  };
};
