cmake_minimum_required(VERSION 3.0.2)
project(hugmy)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  pluginlib
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control pluginlib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
