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
#include <OsqpEigen/OsqpEigen.h>

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
      void reset() override;

      void startRaiseTransition();
      void startLowerLeg();
      void startContactTransition(int leg_id);

      bool getContactTransition();

    private:

      ros::Publisher flight_cmd_pub_; //for spinal
      ros::Publisher gimbal_control_pub_;
      ros::Publisher joint_angle_pub_;
      ros::Publisher joint_torque_pub_;
      ros::Publisher target_vectoring_force_pub_;
      ros::Publisher link_rot_thrust_force_pub_;
      ros::Publisher joint_servo_enable_pub_;
      ros::Subscriber joint_force_compliance_sub_;
      ros::Subscriber joint_no_load_sub_;
      ros::ServiceServer joint_yaw_torque_srv_, joint_pitch_torque_srv_;

      boost::shared_ptr<::Tiger::FullVectoringRobotModel> tiger_robot_model_;
      boost::shared_ptr<aerial_robot_navigation::Tiger::WalkNavigator> tiger_walk_navigator_;

      std::vector<PID> walk_pid_controllers_;
      std::vector<boost::shared_ptr<PidControlDynamicConfig> > walk_pid_reconf_servers_;

      std::vector<int> joint_index_map_;

      Eigen::VectorXd static_thrust_force_;
      Eigen::VectorXd static_joint_torque_;

      std::vector<float> target_base_thrust_;
      std::vector<double> target_gimbal_angles_;
      Eigen::VectorXd target_vectoring_f_;

      sensor_msgs::JointState target_joint_angles_;
      sensor_msgs::JointState target_joint_torques_;
      std::vector<double> prev_navi_target_joint_angles_;
      double joint_ctrl_rate_;
      double tor_kp_;

      bool joint_soft_compliance_;
      double joint_compliance_end_t_;

      bool set_init_servo_torque_;
      bool all_joint_position_control_;
      double joint_torque_control_thresh_;
      double joint_static_torque_limit_;
      double servo_max_torque_;
      double servo_torque_change_rate_;
      double servo_angle_bias_;
      double servo_angle_bias_torque_;

      double angle_scale_;
      double torque_load_scale_;

      double thrust_force_weight_;
      double joint_torque_weight_;

      bool opposite_free_leg_joint_torque_control_mode_;
      bool raise_leg_large_torque_control_;

      double raise_leg_force_i_gain_;
      double modify_leg_force_i_gain_;
      double lower_leg_force_i_gain_;
      double contact_leg_force_i_gain_;
      double lower_leg_force_ratio_thresh_;
      double modify_leg_force_ratio_thresh_;
      double modify_leg_force_margin_;
      double free_leg_force_ratio_;

      Eigen::VectorXd raise_static_thrust_force_;
      double contact_transtion_init_ratio_;
      double lower_leg_speed_;

      double prev_t_;
      double prev_v_;
      double check_interval_;

      bool raise_transition_;
      bool contact_transition_;
      int contact_leg_id_;

      double link_rot_f_control_i_thresh_;
      std::vector<Eigen::Vector3d> fw_i_terms_;

      void rosParamInit();
      virtual void sendCmd() override;

      bool servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& name);
      void jointSoftComplianceCallback(const std_msgs::EmptyConstPtr& msg);

      void cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices) override;

      void calcStaticBalance();
      void jointControl();
      void jointSoftComplianceControl();
      void thrustControl();

      // utils
      void setJointIndexMap();
      std::vector<double> getCurrentJointAngles();
      inline double clamp(double v, double b) { return std::min(std::max(v, -b), b); }
      Eigen::VectorXd clamp(Eigen::VectorXd v, double b);
      bool samejointAngles(std::vector<double> group_a, std::vector<double> group_b);
    };
  };
};
