#include <gtest/gtest.h>
#include <ros/ros.h>

#include <string>

namespace
{
bool waitForParam(const std::string& param_name, const ros::WallDuration& timeout)
{
  const ros::WallTime start = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - start) < timeout)
  {
    if (ros::param::has(param_name))
      return true;
    ros::WallDuration(0.05).sleep();
  }
  return ros::param::has(param_name);
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pinocchio_robot_model_gtest");
  testing::InitGoogleTest(&argc, argv);

  // PinocchioRobotModelRos waits for these parameters forever, so check them
  // here to fail with a readable message instead of hitting the rostest
  // time-limit
  for (const std::string& param_name : { "/robot_description", "/pinocchio_robot_description" })
  {
    if (!waitForParam(param_name, ros::WallDuration(10.0)))
    {
      ROS_FATAL_STREAM("parameter " << param_name << " is not set");
      return 1;
    }
  }

  return RUN_ALL_TESTS();
}
