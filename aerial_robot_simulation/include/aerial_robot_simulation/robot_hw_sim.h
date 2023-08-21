#pragma once

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>
#include <transmission_interface/transmission_info.h>
#include <pluginlib/class_list_macros.h>

#include <aerial_robot_simulation/robot_hw_sim_plugin.h>
#include <aerial_robot_simulation/mujoco_spinal_interface.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

namespace mujoco_ros_control
{
  class MujocoRobotHWSim : public mujoco_ros_control::MujocoRobotHWSimPlugin
  {
  public:
    MujocoRobotHWSim() {};
    ~MujocoRobotHWSim() {}

    bool init(mjModel* mujoco_model,
              mjData* mujoco_data
              );

    void read(const ros::Time& time, const ros::Duration& period);

    void write(const ros::Time& time, const ros::Duration& period);

  protected:
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    hardware_interface::MujocoSpinalInterface spinal_interface_;

  };
}
