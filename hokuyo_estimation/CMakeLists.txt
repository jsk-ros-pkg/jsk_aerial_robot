cmake_minimum_required(VERSION 3.0.2)
project(hokuyo_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
  laser_filters
  filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
message(WARNING "OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nodelet tf std_msgs dynamic_reconfigure geometry_msgs
  LIBRARIES tree_tracking
)

###########
## Build ##
###########
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_library(laser_clustering_filter SHARED src/laser_clustering_filter.cpp)
target_link_libraries(laser_clustering_filter ${catkin_LIBRARIES})

add_library(tree_tracking src/tree_tracking.cpp src/tree_database.cpp src/circle_detection.cpp)
target_link_libraries(tree_tracking ${catkin_LIBRARIES})

add_executable(tree_tracking_node src/tree_tracking_node.cpp)
target_link_libraries (tree_tracking_node ${catkin_LIBRARIES} tree_tracking)

#############
## Install ##
#############
