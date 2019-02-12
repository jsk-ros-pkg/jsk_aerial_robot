/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/*
 Author: Moju Zhao
 Desc: Simulation Wrapper for aerial robot attitude control
*/

#include <aerial_robot_simulation/simulation_attitude_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace flight_controllers {

SimulationAttitudeController::SimulationAttitudeController()
  : loop_count_(0), motor_num_(0), controller_core_(new FlightControl()), debug_mode_(false)
{
  desire_coord_.setRPY(0, 0, 0);
}

bool SimulationAttitudeController::init(hardware_interface::RotorInterface *robot, ros::NodeHandle &n)
{
  rotor_interface_ = robot;
  motor_num_ = rotor_interface_->getNames().size();
  joint_num_ = rotor_interface_->getJointNum();


  controller_core_->init(&n);

  // Set the control gains if necessary
  double gain = 0;
  if(n.getParam("level_p_gain", gain))
    {
      ROS_INFO("simulation_attitude_controller: update level p gain: %f", gain);
      controller_core_->getAttController().levelPGain(gain);
    }
  if(n.getParam("level_i_gain", gain))
    {
      ROS_INFO("simulation_attitude_controller: update level i gain: %f", gain);
      controller_core_->getAttController().levelIGain(gain);
    }
  if(n.getParam("level_d_gain", gain))
    {
      ROS_INFO("simulation_attitude_controller: update level d gain: %f", gain);
      controller_core_->getAttController().levelDGain(gain);
    }
  if(n.getParam("yaw_p_gain", gain))
    {
      ROS_INFO("simulation_attitude_controller: update yaw p gain: %f", gain);
      controller_core_->getAttController().yawPGain(gain);
    }
  if(n.getParam("yaw_i_gain", gain))
    {
      ROS_INFO("simulation_attitude_controller: update yaw i gain: %f", gain);
      controller_core_->getAttController().yawIGain(gain);
    }
  if(n.getParam("yaw_d_gain", gain))
    {
      ROS_INFO("simulation_attitude_controller: update yaw d gain: %f", gain);
      controller_core_->getAttController().yawDGain(gain);
    }

  desire_coord_sub_ = n.subscribe("/desire_coordinate", 1, &SimulationAttitudeController::desireCoordCallback, this);

  debug_sub_ = n.subscribe("/debug_force", 1, &SimulationAttitudeController::debugCallback, this);

  return true;
}

void SimulationAttitudeController::starting(const ros::Time& time)
{
}

void SimulationAttitudeController::update(const ros::Time& time, const ros::Duration& period)
{
  if(!rotor_interface_->foundBaseLink()) return;
  /* update the control coordinate */
  tf::Matrix3x3 orientation = tf::Matrix3x3(rotor_interface_->getBaseLinkOrientation()) * desire_coord_.transpose();
  //ROS_INFO("[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]", orientation.getRow(0).x(), orientation.getRow(0).y(), orientation.getRow(0).z(),orientation.getRow(1).x(), orientation.getRow(1).y(), orientation.getRow(1).z(), orientation.getRow(2).x(), orientation.getRow(2).y(), orientation.getRow(2).z());

  tfScalar r = 0,p = 0, y = 0;
  orientation.getRPY(r,p,y);
  controller_core_->getAttController().setRPY(r,p,y);
  tf::Vector3 w = desire_coord_ * rotor_interface_->getBaseLinkAngular();
  controller_core_->getAttController().setAngular(w.x(), w.y(), w.z());

  //ROS_INFO_THROTTLE(1.0, "cog r, p y: [%f, %f, %f]", r, p, y);
  if(debug_mode_)
    {
      for(int i = 0; i < motor_num_; i++)
        {
          if( i == 0)
            {
              std::stringstream joint_no;
              joint_no << i + 1;
              hardware_interface::RotorHandle rotor = rotor_interface_->getHandle(std::string("rotor") + joint_no.str());
              rotor.setForce(debug_force_);
            }
        }
      return;
    }

  /* update the controller */
  controller_core_->update();

  for(int i = 0; i < motor_num_; i++)
    {
      std::stringstream joint_no;
      joint_no << i + 1;
      hardware_interface::RotorHandle rotor = rotor_interface_->getHandle(std::string("rotor") + joint_no.str());
      rotor.setForce(controller_core_->getAttController().getForce(i));
    }
}



} // namespace

PLUGINLIB_EXPORT_CLASS(flight_controllers::SimulationAttitudeController, controller_interface::ControllerBase)
