/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Author: Moju Zhao
   Desc:   Hardware Interface for aerial robot in Gazebo
*/

#ifndef _GAZEBO_ROS_CONTROL___AERIAL_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___AERIAL_ROBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Rotor Simlation
#include <aerial_robot_model/rotor_interface.h>

namespace gazebo_ros_control
{

class AerialRobotHWSim : public gazebo_ros_control::RobotHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

  const static uint8_t CONTROL_MODE = 0;
  const static uint8_t CMD_VEL_MODE = 1;
  const static uint8_t CMD_POS_MODE = 2;

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, ROTOR};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit);

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  /* Aerial Robot:  for rotor */
  gazebo::physics::ModelPtr parent_model_;
  hardware_interface::RotorInterface    rotor_interface_;
  /* TODO: right now, we only define saturation interface */
  rotor_limits_interface::EffortRotorSaturationInterface er_sat_interface_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;

  uint8_t flight_mode_;
  ros::Subscriber cmd_vel_sub_, cmd_pos_sub_;
  ros::Publisher ground_truth_pub_;
  double ground_truth_pub_rate_;

  geometry_msgs::TwistStamped cmd_vel_;
  geometry_msgs::PoseStamped cmd_pos_;

  /* baselink */
#if GAZEBO_MAJOR_VERSION > 8
  ignition::math::Pose3d base_link_offset_;
#else
   gazebo::math::Pose base_link_offset_;
#endif
  

  std::string base_link_parent_;

  void findBaselinkParent(const KDL::Tree& tree);

  void cmdVelCallback(const geometry_msgs::TwistStampedConstPtr& cmd_vel)
  {
    flight_mode_ = CMD_VEL_MODE;
    cmd_vel_ = *cmd_vel;
  }

  void cmdPosCallback(const geometry_msgs::PoseStampedConstPtr& cmd_pos)
  {
    flight_mode_ = CMD_POS_MODE;
    cmd_pos_ = *cmd_pos;
  }


  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
};

typedef boost::shared_ptr<AerialRobotHWSim> AerialRobotHWSimPtr;

}

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_AERIAL_ROBOT_HW_SIM_H_
