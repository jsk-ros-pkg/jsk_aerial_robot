// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <hydrus/hydrus_robot_model.h>
#include <algorithm>

// optimization libraries
#include <OsqpEigen.h> // osqp-eigen
#include <nlopt.hpp> //nlopt


class QuadVectoringController
{
public:
  QuadVectoringController(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~QuadVectoringController() {}

  inline OsqpEigen::Solver& getYawRangeLPSolver() { return yaw_range_lp_solver_;}
  inline std::shared_ptr<HydrusRobotModel> getRobotModelPtr() const {return robot_model_ptr_;}

  bool verbose_;
  bool simulation_, real_machine_;
  sensor_msgs::JointState joint_state_;
  std::vector<std::string> gimbal_names_;
  std::vector<int> gimbal_index_;
  double max_min_yaw_;
  double force_rate_;
  double yaw_rate_;
  double attitude_thresh_;

protected:

  ros::NodeHandle nh_, nhp_;
  ros::Subscriber actuator_state_sub_;
  ros::Publisher gimbal_ctrl_pub_;
  std::shared_ptr<HydrusRobotModel> robot_model_ptr_;
  OsqpEigen::Solver yaw_range_lp_solver_;
  std::shared_ptr<nlopt::opt> vectoring_nl_solver_;

  double delta_angle_;
  std::vector<double> opt_gimbal_angles_, prev_opt_gimbal_angles_;
  //double max_min_yaw_;

  void actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state);
};
