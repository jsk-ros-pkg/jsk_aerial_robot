cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_dynamics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  )
