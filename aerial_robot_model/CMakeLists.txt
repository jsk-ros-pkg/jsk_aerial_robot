cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  interactive_markers
  visualization_msgs
  tf
  tf2_ros
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  tf_conversions
  eigen_conversions
  kdl_parser
  message_generation
  message_runtime
  spinal
  kalman_filter
)

find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

add_service_files(
  FILES
  AddExtraModule.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES transformable_aerial_robot_model transformable_aerial_robot_model_ros numerical_jacobians
  CATKIN_DEPENDS roscpp  std_msgs  sensor_msgs geometry_msgs tf interactive_markers visualization_msgs message_generation message_runtime spinal eigen_conversions
#  DEPENDS system_lib
)

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

add_library(transformable_aerial_robot_model
  src/transformable_aerial_robot_model/robot_model.cpp
  src/transformable_aerial_robot_model/jacobians.cpp
  src/transformable_aerial_robot_model/kinematics.cpp
  src/transformable_aerial_robot_model/stability.cpp
  src/transformable_aerial_robot_model/statics.cpp)
target_link_libraries(transformable_aerial_robot_model ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${EIGEN3_LIBRARIES})

add_library(transformable_aerial_robot_model_ros src/transformable_aerial_robot_model/robot_model_ros.cpp)
target_link_libraries(transformable_aerial_robot_model_ros transformable_aerial_robot_model ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(transformable_aerial_robot_model_ros ${PROJECT_NAME}_generate_messages_cpp spinal_generate_messages_cpp)

add_library(servo_bridge src/servo_bridge/servo_bridge.cpp)
target_link_libraries(servo_bridge ${catkin_LIBRARIES})
add_executable(servo_bridge_node src/servo_bridge/servo_bridge_node.cpp)
target_link_libraries (servo_bridge_node  ${catkin_LIBRARIES} servo_bridge)

add_executable(rotor_tf_publisher src/utils/rotor_tf_publisher.cpp)
target_link_libraries(rotor_tf_publisher ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(interactive_marker_tf_broadcaster src/utils/interactive_marker_tf_broadcaster.cpp)
target_link_libraries(interactive_marker_tf_broadcaster ${catkin_LIBRARIES})

## for rostest
add_library(numerical_jacobians test/aerial_robot_model/numerical_jacobians.cpp)
target_link_libraries(numerical_jacobians transformable_aerial_robot_model ${catkin_LIBRARIES})


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
