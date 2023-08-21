#include <aerial_robot_simulation/robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool MujocoRobotHWSim::init(mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

  }

  void MujocoRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    
  }

  void MujocoRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    
  }
}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::MujocoRobotHWSim, mujoco_ros_control::MujocoRobotHWSimPlugin);
