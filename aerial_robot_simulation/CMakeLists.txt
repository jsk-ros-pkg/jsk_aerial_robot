cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_simulation)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros_control
  roscpp
  aerial_robot_msgs
  aerial_robot_model
  aerial_robot_base
  tf
  hydrus
)

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aerial_robot_hw_sim flight_controllers
  CATKIN_DEPENDS gazebo_ros_control roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_path(D_BOARD_ATTITUDE_CONTROL_DIRS attitude_control.h ${PROJECT_SOURCE_DIR}/d_board/Jsk_Lib/flight_control/attitude)
#message(WARNING ${D_BOARD_INCLUDE_DIRS})

if(NOT D_BOARD_ATTITUDE_CONTROL_DIRS)
  message(FATAL_ERROR "can not find the headerpath for the d_board attitude control")
else()
  set(D_BOARD_INCLUDE_DIRS d_board/Jsk_Lib)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${D_BOARD_INCLUDE_DIRS}
  include
  )


add_definitions(-DSIMULATION)
add_library(flight_controllers
  d_board/Jsk_Lib/flight_control/attitude/attitude_control.cpp
  src/simulation_attitude_controller.cpp)
target_link_libraries(flight_controllers ${catkin_LIBRARIES})

add_library(aerial_robot_hw_sim src/aerial_robot_hw_sim.cpp)
target_link_libraries(aerial_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aerial_robot_simulation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
