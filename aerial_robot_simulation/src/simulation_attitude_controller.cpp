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
}

bool SimulationAttitudeController::init(hardware_interface::SpinalInterface *robot, ros::NodeHandle &n)
{
  spinal_interface_ = robot;
  motor_num_ = spinal_interface_->getNames().size();
  joint_num_ = spinal_interface_->getJointNum();

  // we have to extract the robot namesace from the ros::controller nodehandle
  int index = n.getNamespace().rfind('/');
  std::string robot_ns = n.getNamespace().substr(0, index);
  ros::NodeHandle n_robot = ros::NodeHandle(robot_ns);
  controller_core_->init(&n_robot, robot->getEstimatorPtr());

  std::string full_param;
  if (n.searchParam("estimation/mode", full_param))
    {
      ROS_DEBUG_STREAM("find " << full_param);

      int estimate_mode;
      n.getParam(full_param, estimate_mode);
      if(estimate_mode == aerial_robot_estimation::GROUND_TRUTH)
        spinal_interface_->useGroundTruth(true);
    }
  else
    {
      ROS_WARN_STREAM_NAMED("simulation_attitude_controller", "can not find rosparam  estimation/mode in ns " << n.getNamespace()  << ", set ground truth mode");
      spinal_interface_->useGroundTruth(true);
    }

  debug_sub_ = n.subscribe("debug_force", 1, &SimulationAttitudeController::debugCallback, this);

  return true;
}

void SimulationAttitudeController::starting(const ros::Time& time)
{
}

void SimulationAttitudeController::update(const ros::Time& time, const ros::Duration& period)
{
  /* freeze the attitude estimator while touching the ground, since the bad contact simulation performance in gazebo */
  spinal_interface_->onGround(!controller_core_->getAttController().getIntegrateFlag());

  if(debug_mode_)
    {
      for(int i = 0; i < motor_num_; i++)
        {
          if( i == 0)
            {
              std::stringstream joint_no;
              joint_no << i + 1;
              hardware_interface::RotorHandle rotor = spinal_interface_->getHandle(std::string("rotor") + joint_no.str());
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
      hardware_interface::RotorHandle rotor = spinal_interface_->getHandle(std::string("rotor") + joint_no.str());
      rotor.setForce(controller_core_->getAttController().getForce(i));
    }
}



} // namespace

PLUGINLIB_EXPORT_CLASS(flight_controllers::SimulationAttitudeController, controller_interface::ControllerBase)
