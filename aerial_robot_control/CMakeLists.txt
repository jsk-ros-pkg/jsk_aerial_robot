cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_model
  aerial_robot_msgs
  dynamic_reconfigure
  geometry_msgs
  pluginlib
  roscpp
  spinal
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_utils
  CATKIN_DEPENDS aerial_robot_model aerial_robot_msgs roscpp spinal
  DEPENDS EIGEN3
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)


### control utils
add_library(control_utils
  src/control/utils/care.cpp
  )
target_link_libraries(control_utils ${EIGEN3_LIBRARIES})

