cmake_minimum_required(VERSION 2.8.3)
project(hydrus_xi)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_model
  aerial_robot_msgs
  angles
  hydrus
  roscpp
  rospy
  spinal
  )

find_package(rostest REQUIRED)

find_package(NLopt REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  hydrus_xi_fully_actuated_robot_model
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs roscpp rospy spinal
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

add_library(hydrus_xi_under_actuated_navigation src/hydrus_xi_under_actuated_navigation.cpp)
target_link_libraries(hydrus_xi_under_actuated_navigation ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(hydrus_xi_fully_actuated_robot_model src/hydrus_xi_fully_actuated_robot_model.cpp)
target_link_libraries(hydrus_xi_fully_actuated_robot_model ${catkin_LIBRARIES})

# pre-build test code
add_executable(hydrus_xi_jacobian_test test/hydrus_xi/jacobian_test.cpp)
target_link_libraries(hydrus_xi_jacobian_test hydrus_xi_fully_actuated_robot_model ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_subdirectory(test)
endif()
