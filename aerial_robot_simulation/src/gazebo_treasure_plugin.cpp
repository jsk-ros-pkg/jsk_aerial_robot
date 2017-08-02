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

#include <aerial_robot_simulation/gazebo_treasure_plugin.h>
#include <string>

namespace gazebo
{

GazeboTreasure::GazeboTreasure()
{
  magnet_on.data = true;
  first_attach_flag_ = false;
}

GazeboTreasure::~GazeboTreasure()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboTreasure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  vel_x = vel_y = vel_yaw = 0;
  static_object_ = false;
  last_time_ = world_->GetSimTime();
  terminated_ = false;

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
    {
      link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
      link_ = _model->GetLink(link_name_);
    }

  if (_sdf->HasElement("staticObject") && _sdf->GetElement("staticObject")->GetValue())
    {
      static_object_ = _sdf->GetElement("staticObject")->Get<std::string>() == "true"?true:false;
    }

  // boost::random::mt19937 rng;
  boost::random::random_device rng;
  boost::random::uniform_int_distribution<> x_rand(-5, 5);
  boost::random::uniform_int_distribution<> y_rand(-5, 5);
  double x = x_rand(rng);
  double y = y_rand(rng);
  model_->SetLinkWorldPose(math::Pose(x, y, 0.2, 0, 0, 0), link_);

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  pub_score_ = node_handle_->advertise<std_msgs::String>("score", 1, true);  // set latch true
  ros::NodeHandle param_handle(*node_handle_, "controller");

  //subscribe to the gazebo models
  gazebo_model_sub = node_handle_->subscribe("/gazebo/model_states",3,&GazeboTreasure::gazebocallback,this);
  magnet_release_sub = node_handle_->subscribe("/mag_on",3,&GazeboTreasure::magnetcallback,this);
  link_state_sub_ = node_handle_->subscribe("/gazebo/link_states", 3, &GazeboTreasure::linkStateCallback, this);
  pub_magnet_get = node_handle_->advertise<std_msgs::Bool>("/gazebo/magnetget",1);
  gazebo_model_pub_ = node_handle_->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);


  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTreasure::Update, this));

}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboTreasure::Update()
{
  if (terminated_) {
    return;
  } else if(magnet_on.data) {
    math::Pose pose_object = link_->GetWorldPose();  //the pose of the object

    //if the uav is close enough, attach to uav
    for(int i = 0; i < this->link_states.name.size(); i++)   //only if when data are received
      {
	//maybe other names, here we use uav
	int j = link_states.name.at(i).find("hydrusx::root");
	int k = link_states.name.at(i).find("hydrusx::link");
	if(j >= 0 || k >= 0) //if find
	  {
	    geometry_msgs::Pose pose_uav = link_states.pose.at(i);
	    if(fabs(pose_uav.position.x - pose_object.pos.x)<0.1 &&
	       fabs(pose_uav.position.y - pose_object.pos.y)<0.1 &&
	       fabs(pose_uav.position.z - pose_object.pos.z)<0.3)
	      {
		//object should follow the uav
		
		gettrue.data = true;
		pub_magnet_get.publish(gettrue);
		gazebo_msgs::ModelState tmpstate;
		tmpstate.model_name = model_->GetName();
		tmpstate.pose = link_states.pose.at(i);
	      	tmpstate.pose.position.z -= 0.2;
		tmpstate.twist = link_states.twist.at(i);
		
		gazebo_model_pub_.publish(tmpstate);
		
	      }
	  } 
      }
    last_time_ = world_->GetSimTime();
    if(gettrue.data) return;   //the object that was picked wont move randomly anymore
    
  }
  else if(gettrue.data) //magnet off
    {
      gettrue.data = false;
      pub_magnet_get.publish(gettrue);
    }
  
  if( static_object_ )
    {
      return;
    }
  
  //  static boost::random::mt19937 rng;
  //  // distribution that maps to 0..9
  //  static boost::random::uniform_int_distribution<> change_direction(0, 299);
  
  //  // normal distribution
  //  static boost::normal_distribution<> nd(0.0, 1.0);
  
  //  static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
  
  
  //  // max 5km/h = 1.38889 = 5 km/h
  //  if ( change_direction(rng) == 0 )
  //  {
  //    vel_x += var_nor();
  //    vel_y += var_nor();
  //    vel_yaw += var_nor();
  //    double vel = sqrt((vel_x*vel_x)+(vel_y*vel_y));
  //    if ( vel > 1.3889 )
  //      {
  //        vel_x *= 1.3389/vel;
  //        vel_y *= 1.3389/vel;
  //      }
  //    if (fabs(vel_yaw) > 1.3389)
  //      {
  //        vel_yaw *= 1.3389/fabs(vel_yaw);
  //      }
  //  }
  
  //  math::Pose pose = link_->GetWorldPose();
  //  // 100 x 60
  //  if ( pose.pos.x > 45 )
  //    {
  //      vel_x = -fabs(vel_x);
  //    }
  
  //  if ( pose.pos.x < -45 )
  //    {
  //      vel_x = fabs(vel_x);
  //    }
  //  if ( pose.pos.y > 25 )
  //    {
  //      vel_y = -fabs(vel_y);
  //    }
  
  //  if ( pose.pos.y < -25 )
  //    {
  //      vel_y = fabs(vel_y);
  //    }
  
  //  model_->SetLinearVel(math::Vector3(vel_x, vel_y, 0));
  //  model_->SetAngularVel(math::Vector3(0, 0, vel_yaw));
  //  last_time_ = world_->GetSimTime();
  
  // check score
}
  
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboTreasure::Reset()
{
  state_stamp_ = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboTreasure)

}  // namespace gazebo
