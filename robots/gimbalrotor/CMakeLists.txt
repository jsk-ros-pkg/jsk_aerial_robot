cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gimbalrotor_robot_model gimbalrotor_controller gimbalrotor_navigation
  CATKIN_DEPENDS   aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
