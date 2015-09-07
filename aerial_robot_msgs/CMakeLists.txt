cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES  DynamicReconfigureLevels.msg  KduinoSimpleImu.msg  KduinoImu.msg RollPitchYawGain.msg FourAxisCommand.msg MotorValue.msg ControlTerm.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)



