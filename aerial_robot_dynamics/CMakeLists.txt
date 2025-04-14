cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_dynamics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  )

add_library(robot_model src/robot_model.cpp src/robot_model_test)
target_link_libraries(robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${pinocchio_LIBRARIES}
  )

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node
  robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${pinocchio_LIBRARIES}
  )
