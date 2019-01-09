cmake_minimum_required(VERSION 2.8.3)
project(hydrus_xi)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_simulation
  aerial_robot_msgs
  angles
  hydrus
  roscpp
  rospy
  spinal
  tf_conversions
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
  cfg/FullyActuatedControllerGains.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_xi_robot_model hydrus_xi_lqi_controller
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

add_library(hydrus_xi_robot_model src/hydrus_xi_robot_model.cpp)
target_link_libraries(hydrus_xi_robot_model ${catkin_LIBRARIES})

add_library(hydrus_xi_lqi_controller src/hydrus_xi_lqi_controller.cpp)
target_link_libraries(hydrus_xi_lqi_controller hydrus_xi_robot_model ${catkin_LIBRARIES})

add_executable(hydrus_xi_lqi_controller_node src/hydrus_xi_lqi_controller_node.cpp)
target_link_libraries(hydrus_xi_lqi_controller_node hydrus_xi_lqi_controller ${catkin_LIBRARIES})

add_library(hydrus_xi_fully_actuated_controller_pluginlib src/hydrus_xi_fully_actuated_controller.cpp)
target_link_libraries(hydrus_xi_fully_actuated_controller_pluginlib hydrus_xi_robot_model ${catkin_LIBRARIES})
add_dependencies(hydrus_xi_fully_actuated_controller_pluginlib ${PROJECT_NAME}_gencfg)
