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
#include <tiger/navigation/walk_navigation.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float32MultiArray.h>
#include <spinal/ServoTorqueCmd.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>

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
      ros::Publisher joint_torque_pub_;
      ros::Subscriber joint_force_compliance_sub_;
      ros::Subscriber joint_no_load_sub_;
      ros::ServiceServer joint_yaw_torque_srv_, joint_pitch_torque_srv_;

      boost::shared_ptr<::Tiger::FullVectoringRobotModel> tiger_robot_model_;
      boost::shared_ptr<aerial_robot_navigation::Tiger::WalkNavigator> tiger_walk_navigator_;

      std::vector<PID> walk_pid_controllers_;
      std::vector<boost::shared_ptr<PidControlDynamicConfig> > walk_pid_reconf_servers_;

      std::vector<float> target_base_thrust_;
      std::vector<double> target_gimbal_angles_;
      Eigen::VectorXd target_vectoring_f_;

      sensor_msgs::JointState target_joint_state_;
      double joint_ctrl_rate_;
      double tor_kp_;

      bool force_joint_control_;
      double joint_no_load_end_t_;

      double joint_torque_control_thresh_;

      void rosParamInit();
      virtual void sendCmd() override;

      bool servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& name);
      void jointForceComplianceCallback(const std_msgs::EmptyConstPtr& msg);
      void jointNoLoadCallback(const std_msgs::EmptyConstPtr& msg);

      void cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices) override;

      void jointControl();
      void thrustControl();
    };
  };
};
