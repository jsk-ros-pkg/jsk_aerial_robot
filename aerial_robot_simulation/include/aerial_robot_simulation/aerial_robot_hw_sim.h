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

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_simulation/noise_model.h>
#include <aerial_robot_simulation/spinal_interface.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <nav_msgs/Odometry.h>

namespace gazebo_ros_control
{

class AerialRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:

  bool initSim(const std::string& robot_namespace,
               ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent_model,
               const urdf::Model *const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

  const static uint8_t FORCE_CONTROL_MODE = 0;
  const static uint8_t SIM_VEL_MODE = 1;
  const static uint8_t SIM_POS_MODE = 2;

protected:
  gazebo::physics::ModelPtr parent_model_;

  /* spinal: rotor handlers */
  unsigned int rotor_n_dof_;
  std::vector<gazebo::physics::JointPtr> sim_rotors_;

  hardware_interface::SpinalInterface spinal_interface_;
  rotor_limits_interface::EffortRotorSaturationInterface er_sat_interface_;

  /* sensor handlers */
  gazebo::sensors::ImuSensorPtr imu_handler_;
  gazebo::sensors::MagnetometerSensorPtr mag_handler_;

  /* baselink */
  std::string baselink_;
#if GAZEBO_MAJOR_VERSION > 8
  ignition::math::Pose3d baselink_offset_;
#else
  gazebo::math::Pose baselink_offset_;
#endif
  std::string baselink_parent_;

  uint8_t control_mode_;
  ros::Subscriber sim_vel_sub_, sim_pos_sub_;
  ros::Publisher ground_truth_pub_;
  ros::Publisher mocap_pub_;
  double ground_truth_pub_rate_;
  double mocap_pub_rate_;

  double mocap_rot_noise_, mocap_pos_noise_;
  double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
  ignition::math::Vector3d ground_truth_rot_curr_drift_, ground_truth_vel_curr_drift_, ground_truth_angular_curr_drift_;
  double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
  double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;

  geometry_msgs::TwistStamped cmd_vel_;
  geometry_msgs::PoseStamped cmd_pos_;

  ros::Time last_ground_truth_time_, last_mocap_time_;

  void cmdVelCallback(const geometry_msgs::TwistStampedConstPtr& cmd_vel)
  {
    control_mode_ = SIM_VEL_MODE;
    cmd_vel_ = *cmd_vel;
  }

  void cmdPosCallback(const geometry_msgs::PoseStampedConstPtr& cmd_pos)
  {
    control_mode_ = SIM_POS_MODE;
    cmd_pos_ = *cmd_pos;
  }
};

typedef boost::shared_ptr<AerialRobotHWSim> AerialRobotHWSimPtr;

}

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_AERIAL_ROBOT_HW_SIM_H_
