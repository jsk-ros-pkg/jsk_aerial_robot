#pragma once

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <aerial_robot_simulation/mujoco/mujoco_visualization_utils.h>
#include <aerial_robot_simulation/mujoco/mujoco_robot_hw_sim_plugin.h>

#include <mujoco/mujoco.h>

namespace mujoco_ros_control
{
  class MujocoRosControl
  {
  public:
    MujocoRosControl(ros::NodeHandle &nh);
    ~MujocoRosControl();

    bool init();
    void update();
    void publishSimTime();
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    bool headless_;

  protected:
    boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::MujocoRobotHWSimPlugin> > robot_hw_sim_loader_;
    boost::shared_ptr<mujoco_ros_control::MujocoRobotHWSimPlugin> robot_hw_sim_;

  private:
    ros::NodeHandle &nh_;

    ros::Time last_update_sim_time_ros_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    ros::Publisher clock_pub_;
    ros::Subscriber control_input_sub_;

    int clock_pub_freq_ = 1000;
    ros::Time last_clock_pub_time_;


  };
}
