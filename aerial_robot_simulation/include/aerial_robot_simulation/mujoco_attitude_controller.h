#pragma once

#include <ros/ros.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_simulation/spinal_interface.h>
#include <aerial_robot_simulation/mujoco_spinal_interface.h>
#include <flight_control/flight_control.h>

class MujocoAttitudeController
{
public:
  MujocoAttitudeController();
  ~MujocoAttitudeController() {}

  void init(boost::shared_ptr<MujocoSpinalInterface> spinal_interface, ros::NodeHandle & nh);
  void starting();
  void update();


private:
  boost::shared_ptr<FlightControl> controller_core_;
  boost::shared_ptr<MujocoSpinalInterface> spinal_interface_;
};
