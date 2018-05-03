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

#ifndef END_EFFECTOR_IK_SOLVER_CORE_H
#define END_EFFECTOR_IK_SOLVER_CORE_H

#include <hydrus/differential_kinematics/planner_core.h>

#include <pluginlib/class_loader.h>
/* special cost for cartesian constraint */
#include <hydrus/differential_kinematics/cost/cartesian_constraint.h>

using namespace differential_kinematics;

class EndEffectorIKSolverCore
{
  using robot_model = TransformController;
public:
  EndEffectorIKSolverCore(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<robot_model> robot_model_ptr);
  ~EndEffectorIKSolverCore(){}

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::ServiceServer end_effector_ik_service_;
  ros::Subscriber actuator_state_sub_;
  tf::TransformBroadcaster br_;

  boost::shared_ptr<Planner> planner_core_ptr_;

  tf::Transform target_ee_pose_;
  sensor_msgs::JointState init_actuator_vector_;

  bool inverseKinematics(const tf::Transform& target_ee_pose, const sensor_msgs::JointState& init_actuator_vector, const tf::Transform& init_root_pose, bool orientation, bool full_body, bool debug);

  void actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state);
  bool endEffectorIkCallback(hydrus::TargetPose::Request  &req,
                             hydrus::TargetPose::Response &res);

  void motionFunc();
};

#endif
