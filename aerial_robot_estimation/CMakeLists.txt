cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_estimation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
message(WARNING "OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(USE_GPU ON)
set(CMAKE_BUILD_TYPE Release)

if(USE_GPU)
  add_definitions(-DUSE_GPU)
endif()

add_library(optical_flow src/optical_flow.cpp)
target_link_libraries(optical_flow ${catkin_LIBRARIES})

add_executable(optical_flow_node src/optical_flow_node.cpp)
target_link_libraries(optical_flow_node ${catkin_LIBRARIES} optical_flow)
