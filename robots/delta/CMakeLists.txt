cmake_minimum_required(VERSION 3.0.2)
project(delta)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  geometry_msgs
  hydrus
  pluginlib
  roscpp
  std_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(OsqpEigen REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3 -g -DNDEBUG")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs geometry_msgs hydrus pluginlib roscpp std_msgs
  LIBRARIES delta_controller delta_robot_model
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  )

add_library(delta_robot_model src/model/delta_robot_model.cpp)
target_link_libraries(delta_robot_model ${catkin_LIBRARIES})

add_library(delta_controller
  src/control/delta_controller_common.cpp
  src/control/delta_flight_controller.cpp
  src/control/delta_wrench_allocation.cpp)
target_link_libraries(delta_controller
  delta_robot_model
  ${catkin_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${NLOPT_LIBRARIES})

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(TARGETS delta_controller delta_robot_model
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_add_env_hooks(99.delta SHELLS bash sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
