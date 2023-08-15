#include <ros/ros.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_simulation/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/mujoco_attitude_controller.h>
#include <aerial_robot_simulation/mujoco_visualization_utils.h>
#include <mujoco/mujoco.h>

class MujocoRobotHWSim
{
public:
  bool initSim(ros::NodeHandle nh
               );
  void readSim(ros::Time time, ros::Duration period);
  void writeSim(ros::Time time, ros::Duration period);

private:
  ros::NodeHandle nh_;
  mjModel* m;
  mjData* d;
  MujocoSpinalInterface spinal_interface_;

};

