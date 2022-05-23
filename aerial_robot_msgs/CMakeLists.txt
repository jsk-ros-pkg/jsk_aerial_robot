cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES
  DynamicReconfigureLevels.msg
  RollPitchNav.msg
  FourAxisGain.msg
  AerialRobotStatus.msg
  FlightNav.msg
  PoseControlPid.msg
  Pid.msg
  State.msg
  States.msg
  Acc.msg
  WrenchAllocationMatrix.msg
  FeasibleControlConvexInfo.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)



