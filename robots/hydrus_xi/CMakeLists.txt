cmake_minimum_required(VERSION 2.8.3)
project(hydrus_xi)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_simulation
  aerial_robot_msgs
  hydrus
  roscpp
  rospy
  spinal
  tf_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_xi_robot_model hydrus_xi_lqi_controller
  CATKIN_DEPENDS aerial_robot_base aerial_robot_simulation aerial_robot_msgs hydrus roscpp rospy spinal tf_conversions
)

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
target_link_libraries(hydrus_xi_fully_actuated_controller_pluginlib ${catkin_LIBRARIES})
#add_dependencies(hydrus_xi_fully_actuated_controller_pluginlib spinal_generate_messages_cpp)
