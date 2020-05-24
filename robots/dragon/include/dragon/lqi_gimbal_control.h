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

#include <hydrus/hydrus_lqi_controller.h>
#include <dragon/dragon_robot_model.h>
#include <dragon/dragon_navigation.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <spinal/RollPitchYawTerm.h>

namespace control_plugin
{
  class DragonLQIGimbalController : public control_plugin::HydrusLQIController
  {
  public:
    DragonLQIGimbalController();
    ~DragonLQIGimbalController(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);
    bool update();
    void reset()
    {
      HydrusLQIController::reset();

      yaw_control_terms_.assign(1, 0);

      level_flag_ = false;
      landing_flag_ = false;
    }

    void sendCmd();
  private:
    ros::Publisher gimbal_control_pub_;
    ros::Publisher gimbal_target_force_pub_;
    ros::Publisher  roll_pitch_pid_pub_;
    ros::Subscriber att_control_feedback_state_sub_;
    ros::Subscriber extra_vectoring_force_sub_;

    void gimbalControl();
    void rosParamInit();

    void attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg);
    void extraVectoringForceCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

    boost::shared_ptr<DragonRobotModel> dragon_robot_model_;
    std::vector<double> target_thrust_terms_; // the scalar value of vectoring force: ||f||
    Eigen::MatrixXd P_xy_;

    /* target baselink rotation */
    std::vector<double> target_gimbal_angles_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* pitch roll control */
    double gimbal_pitch_roll_control_rate_thresh_;
    double gimbal_pitch_roll_control_p_det_thresh_;
    tf::Vector3 gimbal_pitch_roll_gains_;
    double gimbal_pitch_roll_limit_;
    tf::Vector3 gimbal_pitch_roll_terms_limits_;
    double gimbal_roll_i_term_, gimbal_pitch_i_term_;
    double gimbal_roll_control_stamp_;
    double gimbal_pitch_control_stamp_;
    bool gimbal_vectoring_check_flag_;

    bool add_lqi_result_;
    std::vector<double> lqi_att_terms_;

    /* landing process */
    bool level_flag_;
    bool landing_flag_;
    bool servo_torque_;

    /* external wrench */
    ros::ServiceServer add_external_wrench_service_, clear_external_wrench_service_;
    bool addExternalWrenchCallback(gazebo_msgs::ApplyBodyWrench::Request& req, gazebo_msgs::ApplyBodyWrench::Response& res);
    bool clearExternalWrenchCallback(gazebo_msgs::BodyRequest::Request& req, gazebo_msgs::BodyRequest::Response& res);

    /* extra vectoring force (i.e., for grasping) */
    Eigen::VectorXd extra_vectoring_force_;

    /* rosparam */
    bool control_verbose_;
    double height_thresh_;
    string joints_torque_control_srv_name_, gimbals_torque_control_srv_name_;
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;

    /* cfg */
    boost::shared_ptr<dynamic_reconfigure::Server<aerial_robot_control::XYPidControlConfig> > gimbal_roll_pitch_pid_server_;
    dynamic_reconfigure::Server<aerial_robot_control::XYPidControlConfig>::CallbackType dynamic_reconf_func_gimbal_roll_pitch_pid_;
    void cfgGimbalPitchRollPidCallback(aerial_robot_control::XYPidControlConfig &config, uint32_t level);

    boost::shared_ptr<dynamic_reconfigure::Server<aerial_robot_control::XYPidControlConfig> > gimbal_yaw_pid_server_;
    dynamic_reconfigure::Server<aerial_robot_control::XYPidControlConfig>::CallbackType dynamic_reconf_func_gimbal_yaw_pid_;
    void cfgGimbalYawPidCallback(aerial_robot_control::XYPidControlConfig &config, uint32_t level);
  };
};
