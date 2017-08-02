/*
 * Copyright (c) 2017, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AERIAL_ROBOT_SIMULATION_GAZEBO_TREASURE_PLUGIN_H
#define AERIAL_ROBOT_SIMULATION_GAZEBO_TREASURE_PLUGIN_H

#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Bool.h>

#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <geometry_msgs/Point.h>

namespace gazebo
{

class GazeboTreasure : public ModelPlugin
{
public:
  GazeboTreasure();
  virtual ~GazeboTreasure();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  physics::ModelPtr model_;

  std::string link_name_;
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::Publisher pub_score_;

  ros::Time state_stamp_;

  event::ConnectionPtr update_connection_;

  //treasure plugin for attaching the object to uav
  ros::Subscriber gazebo_model_sub; //get the model states
  ros::Subscriber magnet_release_sub; //release the object(disable attach)
  ros::Subscriber link_state_sub_;
  ros::Publisher pub_magnet_get;
  ros::Publisher gazebo_model_pub_;

  gazebo_msgs::ModelStates gazebo_models; //model states...
  gazebo_msgs::LinkStates link_states;
  std_msgs::Bool magnet_on;
  std_msgs::Bool gettrue; //get the object

  double vel_x, vel_y, vel_yaw;
  common::Time last_time_;
  bool terminated_;
  bool static_object_;
  bool first_attach_flag_;
  geometry_msgs::Point uav_object_offset_;
  
  //renew the data of the gazebo objects
  void gazebocallback(const gazebo_msgs::ModelStates gazebo_model_states)
  {
    this->gazebo_models = gazebo_model_states;
  }

  void linkStateCallback(const gazebo_msgs::LinkStates msg)
  {
    this->link_states = msg;
  }
 
  void magnetcallback(const std_msgs::Bool on)
  {
    this->magnet_on = on;
  }
};

}  // namespace gazebo

#endif  // AERIAL_ROBOT_SIMULATION_GAZEBO_TREASURE_PLUGIN_H
