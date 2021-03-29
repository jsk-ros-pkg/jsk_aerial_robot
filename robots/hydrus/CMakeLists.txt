cmake_minimum_required(VERSION 2.8.3)
project(hydrus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  spinal
  eigen_conversions
  tf_conversions
  std_srvs
  dynamic_reconfigure
  )

find_package(Eigen3 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

generate_dynamic_reconfigure_options(
  cfg/LQI.cfg)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES hydrus_robot_model hydrus_controller_pluginlib hydrus_numerical_jacobians
  CATKIN_DEPENDS roscpp aerial_robot_control aerial_robot_model aerial_robot_msgs spinal tf_conversions eigen_conversions
  DEPENDS EIGEN3
)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  test
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(hydrus_robot_model
  src/hydrus_robot_model.cpp
  src/hydrus_tilted_robot_model.cpp)
target_link_libraries(hydrus_robot_model ${catkin_LIBRARIES})

### flight control plugin
add_library(hydrus_controller_pluginlib
  src/hydrus_lqi_controller.cpp
  src/hydrus_tilted_lqi_controller.cpp
  src/hydrus_tilted_lqi_adaptive_controller.cpp
  )
target_link_libraries(hydrus_controller_pluginlib hydrus_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(hydrus_controller_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp ${PROJECT_NAME}_gencfg)

# test
# pre-build test code
add_library(hydrus_numerical_jacobians test/hydrus/numerical_jacobians.cpp)
target_link_libraries(hydrus_numerical_jacobians hydrus_robot_model ${catkin_LIBRARIES})
add_executable(hydrus_jacobian_test test/hydrus/jacobian_test.cpp)
target_link_libraries(hydrus_jacobian_test hydrus_numerical_jacobians ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_subdirectory(test)
endif()

