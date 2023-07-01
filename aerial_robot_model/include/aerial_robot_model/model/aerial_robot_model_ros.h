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

#pragma once

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_model/AddExtraModule.h>
#include <pluginlib/class_loader.h>
#include <spinal/DesireCoord.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


namespace aerial_robot_model {

  //Transformable Aerial Robot Model with ROS functions
  class RobotModelRos {
  public:
    RobotModelRos(ros::NodeHandle nh, ros::NodeHandle nhp);
    virtual ~RobotModelRos() = default;

    //public functions
    sensor_msgs::JointState getJointState() const { return joint_state_; }

    const boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModel() const { return robot_model_; }

  private:
    //private attributes
    ros::ServiceServer add_extra_module_service_;
    ros::Subscriber desire_coordinate_sub_;
    ros::Subscriber joint_state_sub_;
    tf2_ros::TransformBroadcaster br_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    sensor_msgs::JointState joint_state_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
    std::string tf_prefix_;

    //private functions
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& state);
    bool addExtraModuleCallback(aerial_robot_model::AddExtraModule::Request& req, aerial_robot_model::AddExtraModule::Response& res);
    void desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg);
  };
} //namespace aerial_robot_model
