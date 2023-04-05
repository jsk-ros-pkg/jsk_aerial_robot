cmake_minimum_required(VERSION 3.0.2)
project(birotor)

add_compile_options(-std=c++14)

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
  # LIBRARIES birotor_robot_model
  CATKIN_DEPENDS   aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(birotor_controller src/control/birotor_controller.cpp)
target_link_libraries(birotor_controller ${catkin_LIBRARIES})

add_library(birotor_robot_model src/model/birotor_robot_model.cpp)
target_link_libraries(birotor_robot_model ${catkin_LIBRARIES})
