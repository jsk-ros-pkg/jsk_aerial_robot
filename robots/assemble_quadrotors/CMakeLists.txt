cmake_minimum_required(VERSION 2.8.3)
project(assemble_quadrotors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  spinal
  )

find_package(Eigen3 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

catkin_package(
  CATKIN_DEPENDS roscpp aerial_robot_control aerial_robot_model aerial_robot_msgs spinal
  DEPENDS EIGEN3
)

###########
## Build ##
###########
