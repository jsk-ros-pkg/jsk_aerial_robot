cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_model)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  interactive_markers
  kalman_filter
  kdl_parser
  message_generation
  pluginlib
  sensor_msgs
  spinal
  std_msgs
  tf
  tf_conversions
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  tf_conversions
  urdf
  visualization_msgs
  )

find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})

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
  LIBRARIES aerial_robot_model aerial_robot_model_ros numerical_jacobians robot_model_pluginlib
  CATKIN_DEPENDS eigen_conversions geometry_msgs interactive_markers kalman_filter kdl_parser message_runtime pluginlib sensor_msgs spinal std_msgs tf tf_conversions tf2_eigen tf2_geometry_msgs tf2_kdl tf2_ros tf_conversions urdf visualization_msgs
  DEPENDS orocos_kdl urdfdom_headers
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


add_library(aerial_robot_model
  src/model/base_model/robot_model.cpp
  src/model/transformable_model/robot_model.cpp
  src/model/transformable_model/jacobians.cpp
  src/model/transformable_model/kinematics.cpp
  src/model/transformable_model/stability.cpp
  src/model/transformable_model/statics.cpp)
target_link_libraries(aerial_robot_model ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${EIGEN3_LIBRARIES})

add_library(aerial_robot_model_ros src/model/base_model/robot_model_ros.cpp)
target_link_libraries(aerial_robot_model_ros aerial_robot_model ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(aerial_robot_model_ros ${PROJECT_NAME}_generate_messages_cpp spinal_generate_messages_cpp)

add_library(robot_model_pluginlib
  src/model/plugin/multirotor_robot_model.cpp
  src/model/plugin/tilted_underactuated_robot_model.cpp)
target_link_libraries(robot_model_pluginlib ${catkin_LIBRARIES} aerial_robot_model)

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
target_link_libraries(numerical_jacobians aerial_robot_model ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/ test/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS aerial_robot_model aerial_robot_model_ros robot_model_pluginlib servo_bridge numerical_jacobians
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS rotor_tf_publisher interactive_marker_tf_broadcaster servo_bridge_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch script plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
