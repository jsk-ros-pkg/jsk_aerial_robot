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

#ifndef GAP_PASSING_SOLVER_H
#define GAP_PASSING_SOLVER_H

#include <hydrus/differential_kinematics/planner_core.h>

#include <pluginlib/class_loader.h>
/* special cost plugin for cartesian constraint */
#include <hydrus/differential_kinematics/cost/cartesian_constraint.h>
/* special constraint plugin for collision avoidance */
#include <hydrus/differential_kinematics/constraint/collision_avoidance.h>

/* utils */
#include <tf_conversions/tf_kdl.h>

using namespace differential_kinematics;

enum Phase
  {
    CASE1,
    CASE2_1,
    CASE2_2,
    CASE3,
  };

class GapPassingSolver
{
  using robot_model = TransformController;
public:
  GapPassingSolver(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<robot_model> robot_model_ptr);
  ~GapPassingSolver(){}

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber actuator_state_sub_;
  ros::Publisher env_collision_pub_;
  tf::TransformBroadcaster br_;

  boost::shared_ptr<robot_model> robot_model_ptr_;
  boost::shared_ptr<Planner> planner_core_ptr_;

  sensor_msgs::JointState init_actuator_vector_;
  tf::Transform init_root_pose_;
  tf::Transform openning_center_frame_;

  double delta_pinch_length_; //to propagate the pinch action

  bool debug_;
  Phase phase_;
  double reference_point_ratio_;

  boost::shared_ptr<cost::CartersianConstraint<Planner> > cartersian_constraint_;

  visualization_msgs::MarkerArray env_collision_;

  bool solver(bool debug);

  void actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state);

  void motionFunc();
  bool updatePinchPoint();
};

#endif
