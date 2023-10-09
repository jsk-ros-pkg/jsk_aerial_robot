#pragma once

#include <ros/ros.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <flight_control/flight_control.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>

namespace flight_controllers
{
  class MujocoAttitudeController : public controller_interface::Controller<hardware_interface::MujocoSpinalInterface>
  {
  public:
    MujocoAttitudeController();
    ~MujocoAttitudeController() {}

    bool init(hardware_interface::MujocoSpinalInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    hardware_interface::MujocoSpinalInterface* spinal_interface_;
    boost::shared_ptr<FlightControl> controller_core_;
    int motor_num_;

  };
}
