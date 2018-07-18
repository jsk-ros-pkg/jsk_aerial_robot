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

#include <aerial_robot_model/transformable_aerial_robot_model.h>
#include <spinal/DesireCoord.h>
#include <aerial_robot_model/AddExtraModule.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

namespace aerial_robot_model {

  //Transformable Aerial Robot Model with ROS functions
  class RobotModelRos {
  public:
    RobotModelRos(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<aerial_robot_model::RobotModel> robot_model = std::make_unique<aerial_robot_model::RobotModel>(true));
    virtual ~RobotModelRos() = default;

    bool getKinematicsUpdated() const
    {
      return kinematics_updated_;
    }
  protected:
    aerial_robot_model::RobotModel& getRobotModel() const
    {
      return *robot_model_;
    }

  private:
    std::unique_ptr<aerial_robot_model::RobotModel> robot_model_;

    ros::NodeHandle nh_, nhp_;
    ros::Publisher cog2baselink_tf_pub_;
    ros::Subscriber actuator_state_sub_;
    ros::Subscriber desire_coordinate_sub_;
    ros::ServiceServer add_extra_module_service_;
    tf2_ros::TransformBroadcaster br_;

    bool kinematics_updated_;
    void actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state);
    void desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg);
    bool addExtraModuleCallback(aerial_robot_model::AddExtraModule::Request& req, aerial_robot_model::AddExtraModule::Response& res);
  };
} //namespace aerial_robot_model
