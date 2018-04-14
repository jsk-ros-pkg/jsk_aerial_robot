cmake_minimum_required(VERSION 2.8.3)
project(dragon)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  aerial_robot_msgs
  aerial_robot_base
  hydrus
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
message(STATUS "CMAKE_BUILD_TYPE:               ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE:        ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_CXX_FLAGS_MINSIZEREL:     ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:          ${CMAKE_CXX_FLAGS_DEBUG}")

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dragon_transform_control
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Eigen3
)

cmake_policy(SET CMP0046 OLD)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $
)

## Declare a C++ library
add_library(dragon_transform_control
  src/transform_control.cpp
)
target_link_libraries (dragon_transform_control ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(dragon_transform_control aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp hydrus_generate_messages_cpp hydrus_gencfg)


add_executable (dragon_transform_control_node src/transform_control_node.cpp)
target_link_libraries (dragon_transform_control_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} dragon_transform_control)
add_dependencies(dragon_transform_control_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp hydrus_generate_messages_cpp hydrus_gencfg)


add_library(dragon_control_pluginlib src/gimbal_control.cpp)
target_link_libraries (dragon_control_pluginlib ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} dragon_transform_control)
add_dependencies(dragon_control_pluginlib aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp hydrus_generate_messages_cpp hydrus_gencfg)

add_library (dragon_dynamixel_bridge src/dynamixel_bridge.cpp)
target_link_libraries (dragon_dynamixel_bridge ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(dragon_dynamixel_bridge aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable(dynamixel_bridge_node src/dynamixel_bridge_node.cpp)
target_link_libraries (dynamixel_bridge_node  ${catkin_LIBRARIES} dragon_dynamixel_bridge)
add_dependencies(dynamixel_bridge_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
