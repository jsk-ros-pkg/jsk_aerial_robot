cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_base)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  roscpp
  rospy)

catkin_python_setup()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library (aerial_robot_base src/aerial_robot_base.cpp)
target_link_libraries (aerial_robot_base ${catkin_LIBRARIES})

add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
target_link_libraries (aerial_robot_base_node ${catkin_LIBRARIES} aerial_robot_base)
