// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#ifndef DIFFERENTIAL_KINEMATICS_PLANNER_H
#define DIFFERENTIAL_KINEMATICS_PLANNER_H

/* ros */
#include <ros/ros.h>

/* robot model */
#include <hydrus/transform_control.h>

/* for QP solution for force-closure */
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

/* cost & constraint */
#include <hydrus/differential_kinematics/cost/base_plugin.h>
#include <hydrus/differential_kinematics/constraint/base_plugin.h>


namespace differential_kinematics
{
  class Planner
  {
    using RobotModel = TransformController;
    using CostContainer = std::vector<boost::shared_ptr<cost::Base<Planner> > >;
    using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base<Planner> > >;

  public:
    Planner(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<RobotModel> robot_model_ptr );
    ~Planner(){}

    bool solver(CostContainer cost_container, ConstraintContainer constraint_container, bool debug);

    void registerUpdateFunc(std::function<void(void)> new_func);
    void registerMotionFunc(std::function<void(void)> new_func);

    static constexpr uint8_t MULTILINK_TYPE_SE2 = 0;
    static constexpr uint8_t MULTILINK_TYPE_SE3 = 1;

    /* kinematics */
    boost::shared_ptr<RobotModel> getRobotModelPtr() {return robot_model_ptr_;}
    const sensor_msgs::JointState& getTargetActuatorVector() {return target_actuator_vector_;}
    const tf::Transform& getTargetRootPose() {return target_root_pose_;}
    const Eigen::VectorXd getTargetJointVector();
    const int getMultilinkType() {return multilink_type_;}

    inline void setInitRootPose(const tf::Transform& init_root_pose) { target_root_pose_ = init_root_pose;}
    inline void setInitActuatorPose(const sensor_msgs::JointState& init_actuator_vector) {target_actuator_vector_ = init_actuator_vector;}

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher actuator_state_pub_;
    tf::TransformBroadcaster br_;
    ros::Timer  motion_timer_;

    /* robot model for kinematics */
    boost::shared_ptr<RobotModel>  robot_model_ptr_;
    uint8_t multilink_type_;

    /* result  */
    sensor_msgs::JointState target_actuator_vector_;
    tf::Transform target_root_pose_;
    bool solved_;
    int differential_kinematics_count_;

    int sequence_;
    std::vector<sensor_msgs::JointState> target_actuator_vector_sequence_;
    std::vector<tf::Transform> target_root_pose_sequence_;


    /* update function for each loop of the differential kinematics */
    std::vector< std::function<void(void)> > update_func_vector_;
    /* update function for the result motion, mainly for visualization */
    std::vector< std::function<void(void)> > motion_func_vector_;

    void motionFunc(const ros::TimerEvent & e);
  };
};

#endif
