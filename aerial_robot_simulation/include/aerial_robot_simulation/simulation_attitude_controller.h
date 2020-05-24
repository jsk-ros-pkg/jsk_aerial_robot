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

#ifndef SIMULATION_ATTITUDE_CONTROLLER_H
#define SIMULATION_ATTITUDE_CONTROLLER_H

#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_simulation/spinal_interface.h>
#include <boost/scoped_ptr.hpp>
#include <controller_interface/controller.h>
#include <flight_control/flight_control.h>
#include <ros/node_handle.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Transform.h>
#include <urdf/model.h>

namespace flight_controllers
{

class SimulationAttitudeController: public controller_interface::Controller<hardware_interface::SpinalInterface>
{
public:

  struct Commands
  {
    double pwm_; // Last commanded pwm [0 ~ 1])
  };

  SimulationAttitudeController();
  ~SimulationAttitudeController() {}

  bool init(hardware_interface::SpinalInterface *robot, ros::NodeHandle &n);

  void setCommand(double pos_target);
  void setCommand(double pos_target, double vel_target);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
  void printDebug();
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
  std::string getJointName();
  double getPosition();

private:
  ros::Subscriber debug_sub_;
  int loop_count_;

  hardware_interface::SpinalInterface* spinal_interface_;
  boost::shared_ptr<FlightControl> controller_core_;
  uint8_t motor_num_;
  uint8_t joint_num_;

  bool debug_mode_;
  float debug_force_;

  void debugCallback(const std_msgs::Float64& debug_force)
  {
    debug_mode_ = true;
    debug_force_ = debug_force.data;
  }

};

} // namespace

#endif
