#include <aerial_robot_dynamics/test_utils.h>
#include <aerial_robot_dynamics/robot_model_ros.h>

#include <ros/ros.h>

namespace aerial_robot_dynamics
{
PinocchioRobotModel& getTestRobotModel()
{
  ros::NodeHandle nh;
  static PinocchioRobotModelRos robot_model_ros(nh);
  return *robot_model_ros.getPinocchioRobotModel();
}

bool testVerbose()
{
  static const bool verbose = ros::NodeHandle("~").param("verbose", false);
  return verbose;
}

int testTrials()
{
  static const int trials = ros::NodeHandle("~").param("trials", 3);
  return trials;
}
}  // namespace aerial_robot_dynamics
