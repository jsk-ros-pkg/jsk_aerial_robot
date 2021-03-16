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

#include <ros/ros.h>
#include <vector>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <unsupported/Eigen/MatrixFunctions>

class TorsionEstimator {
public:
  TorsionEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~TorsionEstimator() {}

private:
  //private functions
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher estimate_state_pub_;
  ros::Timer kf_timer_;
  double kf_step_rate_;
  void kfStepCallback(const ros::TimerEvent&);

  bool debug_;
  bool is_simulation_;
  bool is_use_mocap_;

  std::string robot_name_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  std::vector<geometry_msgs::TransformStamped> link_transforms_;

  int link_num_;
  std::string neuron_imu_frame_name_prefix_;
  std::string neuron_imu_frame_name_suffix_;
  std::vector<ros::Subscriber> neuron_imu_subs_;
  std::vector<sensor_msgs::Imu> neuron_imu_data_;
  void neuronIMUCallback(const sensor_msgs::ImuConstPtr& msg, const int link_id);
  void mocapCallback(const geometry_msgs::PoseStampedConstPtr& msg, const int link_id);
  void simMocapCallback(const nav_msgs::OdometryConstPtr& msg, const int link_id);
  void mocapFilter(const int link_id, const geometry_msgs::Pose& pose, const std_msgs::Header& header);

  double torsion_vel_cutoff_freq_;
  double torsion_vel_q_;
  std::vector<double> torsion_vel_in_prev1_;
  std::vector<double> torsion_vel_in_prev2_;
  std::vector<double> torsion_vel_out_prev1_;
  std::vector<double> torsion_vel_out_prev2_;
  double torsion_cutoff_freq_;
  double torsion_q_;
  std::vector<double> torsion_in_prev1_;
  std::vector<double> torsion_in_prev2_;
  std::vector<double> torsion_out_prev1_;
  std::vector<double> torsion_out_prev2_;
  std::vector<Eigen::Matrix3d> R_ci_cache_;
  std::vector<double> prev_torsions_;

  void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, tf::Quaternion quat){
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
  }
  void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
  }
  Eigen::Matrix3d quat_to_rot_mat(double& x, double& y, double& z, double&w) {
    Eigen::Matrix3d result;
    result(0,0)= x*x-y*y-z*z+w*w;result(0,1)= 2.0*(x*y-w*z);  result(0,2)= 2.0*(x*z+w*y);
    result(1,0)= 2.0*(x*y+w*z);  result(1,1)= y*y+w*w-x*x-z*z;result(1,2)= 2.0*(y*z-w*x);
    result(2,0)= 2.0*(x*z-w*y);  result(2,1)= 2.0*(y*z+w*x);  result(2,2)= z*z+w*w-x*x-y*y;
    return result;
  }
  Eigen::Matrix3d quat_to_rot_mat(geometry_msgs::Quaternion& q) {
    return quat_to_rot_mat(q.x, q.y, q.z, q.w);
  }
};

