#pragma once

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>
#include <transmission_interface/transmission_info.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

namespace mujoco_ros_control
{
  class MujocoRobotHWSimPlugin : public hardware_interface::RobotHW
  {
  public:
    virtual ~MujocoRobotHWSimPlugin() {}

    virtual bool init(const std::string& robot_namespace,
                      ros::NodeHandle model_nh,
                      mjModel* mujoco_model,
                      mjData* mujoco_data) = 0;
  };
}

