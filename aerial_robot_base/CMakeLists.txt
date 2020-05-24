cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_base)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  spinal
  tf_conversions
  visualization_msgs)

catkin_python_setup()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

generate_dynamic_reconfigure_options(
  cfg/XYPidControl.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} flight_control_pluginlib
  CATKIN_DEPENDS roscpp aerial_robot_estimation aerial_robot_model aerial_robot_msgs geometry_msgs visualization_msgs nav_msgs pluginlib
)

### Temporary soulition because of https://github.com/ros/ros_comm/issues/1404
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)|(armhf)" AND $ENV{ROS_DISTRO} MATCHES "melodic")
  message(WARNING "please check https://github.com/ros/ros_comm/issues/1404")
  add_definitions (-DARM_MELODIC)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


### flight control plugin
add_library(flight_control_pluginlib
  src/control/flatness_pid_controller.cpp
  src/control/agile_flatness_pid_controller.cpp)
target_link_libraries(flight_control_pluginlib ${catkin_LIBRARIES})
add_dependencies(flight_control_pluginlib  ${PROJECT_NAME}_gencfg)

add_library (aerial_robot_base src/aerial_robot_base.cpp src/flight_navigation.cpp)
target_link_libraries (aerial_robot_base ${catkin_LIBRARIES})

add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
target_link_libraries (aerial_robot_base_node ${catkin_LIBRARIES} aerial_robot_base)
