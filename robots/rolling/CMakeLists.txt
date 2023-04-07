cmake_minimum_required(VERSION 3.0.2)
project(rolling)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
