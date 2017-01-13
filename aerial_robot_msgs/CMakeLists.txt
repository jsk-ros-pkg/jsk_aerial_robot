cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES  DynamicReconfigureLevels.msg SimpleImu.msg FourAxisCommand.msg  ControlTerm.msg Imu.msg Barometer.msg Gps.msg FourAxisGain.msg)

add_service_files(FILES BoolFlag.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)



