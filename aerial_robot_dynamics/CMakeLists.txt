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
find_package(urdfdom_headers REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -DNDEBUG")

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
    src/test/tauext_by_thrust_derivative_test.cpp
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS robot_model
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
