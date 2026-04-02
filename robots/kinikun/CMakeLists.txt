cmake_minimum_required(VERSION 3.0.2)
project(kinikun)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  diagnostic_msgs
  pluginlib
  roscpp
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  config/PID.cfg
  config/Angle_PID.cfg
  config/NarxController.cfg
  config/RelativePID.cfg
)

catkin_package(
  CATKIN_DEPENDS
    aerial_robot_control
    aerial_robot_estimation
    aerial_robot_model
    dynamic_reconfigure
    diagnostic_msgs
    roscpp
    aerial_robot_msgs
  INCLUDE_DIRS include
  LIBRARIES kinikun_controller_pluginlib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(kinikun_controller_pluginlib
  src/kinikun_tilted_lqi_controller.cpp
)
target_link_libraries(kinikun_controller_pluginlib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(kinikun_controller_pluginlib
  aerial_robot_msgs_generate_messages_cpp
  spinal_generate_messages_cpp
)

install(TARGETS kinikun_controller_pluginlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch scripts plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)