cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_dynamics)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(urdfdom_headers REQUIRED)

if(OPENMP_FOUND)
  message(STATUS "OpenMP found.")
  MESSAGE("OpenMP found.")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -DNDEBUG")

option(BUILD_TEST "Build Test Code" ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  )

add_library(robot_model src/robot_model.cpp)
target_link_libraries(robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${pinocchio_LIBRARIES}
  )

if(BUILD_TEST)
  add_library(robot_model_test
    src/test/fd_test.cpp
    src/test/fd_derivative_test.cpp
    src/test/id_test.cpp
    src/test/id_derivative_test.cpp
    src/test/tauext_by_thrust_q_derivative_test.cpp
    src/test/tauext_by_thrust_derivative_q_derivative_test.cpp
    )
  target_link_libraries(robot_model_test
    robot_model
    ${catkin_LIBRARIES}
    ${OsqpEigen_LIBRARIES}
    ${pinocchio_LIBRARIES}
    )

  add_executable(test_node src/test/test_node.cpp)
  target_link_libraries(test_node
    robot_model
    robot_model_test
    ${catkin_LIBRARIES}
    ${OsqpEigen_LIBRARIES}
    ${pinocchio_LIBRARIES}
    )
endif()
