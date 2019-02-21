cmake_minimum_required(VERSION 3.2)
project(hydrus_xi)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_simulation
  aerial_robot_msgs
  angles
  hydrus
  roscpp
  ros_numpy
  rospy
  spinal
  tf_conversions
  dynamic_reconfigure
  )

# OSQP + OSQP-eigen
add_subdirectory(osqp)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp/osqp/build/out)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp/osqp/include)

#add_subdirectory(osqp-eigen)
add_subdirectory(osqp-eigen)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp-eigen/osqp-eigen/build)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/osqp-eigen/osqp-eigen/include)

# NLOPT
add_subdirectory(nlopt)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/nlopt/nlopt/build)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/nlopt/nlopt/build/src/api)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/FullyActuatedControllerGains.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_xi_quad_vectoring_controller
  CATKIN_DEPENDS aerial_robot_base aerial_robot_simulation aerial_robot_msgs hydrus roscpp rospy spinal tf_conversions dynamic_reconfigure
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
message(STATUS "CMAKE_BUILD_TYPE:               ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE:        ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_CXX_FLAGS_MINSIZEREL:     ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:          ${CMAKE_CXX_FLAGS_DEBUG}")

cmake_policy(SET CMP0046 OLD)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hydrus_xi_quad_vectoring_controller src/hydrus_xi_quad_vectoring_controller.cpp)
target_link_libraries(hydrus_xi_quad_vectoring_controller ${catkin_LIBRARIES} OsqpEigen osqp nlopt)

add_executable(hydrus_xi_quad_vectoring_controller_node src/hydrus_xi_quad_vectoring_controller_node.cpp)
target_link_libraries(hydrus_xi_quad_vectoring_controller_node hydrus_xi_quad_vectoring_controller ${catkin_LIBRARIES})
