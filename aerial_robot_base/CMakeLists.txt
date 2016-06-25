cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_base)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  visualization_msgs
  tf
  dynamic_reconfigure
  aerial_robot_msgs
  kalman_filter
  cmake_modules
  pluginlib
)


find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_message_files(FILES
  AerialRobotStatus.msg
  FlightNav.msg
  FourAxisPid.msg
  ImuData.msg
  ImuQu.msg
  OpticalFlow.msg
  Pid.msg
  State.msg
  States.msg
  Pwms.msg
  YawThrottleGain.msg
  DesireCoord.msg
  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

generate_dynamic_reconfigure_options(
  cfg/XYPidControl.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime aerial_robot_msgs std_msgs geometry_msgs visualization_msgs nav_msgs  sensor_msgs laser_geometry tf kalman_filter pluginlib
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

## pluginlib_tutorials library
add_library(sensor_pluginlib src/sensor_plugin.cpp)
target_link_libraries(sensor_pluginlib ${catkin_LIBRARIES})

add_library (aerial_robot_base src/aerial_robot_base.cpp
  src/flight_navigation.cpp
  src/flight_control.cpp
  src/state_estimation.cpp
)
add_dependencies(aerial_robot_base aerial_robot_msgs_gencpp ${PROJECT_NAME}_gencpp)
## important link
target_link_libraries (aerial_robot_base ${catkin_LIBRARIES}  ${Boost_LIBRARIES}) 


add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
target_link_libraries (aerial_robot_base_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} aerial_robot_base) 
add_dependencies(aerial_robot_base_node aerial_robot_msgs_generate_message_cpp ${PROJECT_NAME}_generate_message_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(aerial_robot_base_node  ${PROJECT_NAME}_gencfg)

#add_executable(rotate_node src/rotate.cpp)
#target_link_libraries (rotate_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} aerial_robot_base) 

#add_executable(mocap_node src/mocap_node.cpp)
#target_link_libraries (mocap_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES}) 


## sensor plugin_loader test
#add_executable(sensor_plugin_test src/sensor_plugin_test.cpp)
#target_link_libraries(sensor_plugin_test ${catkin_LIBRARIES})



#add_executable(timestamp_modification src/timestamp_modification.cpp)
#target_link_libraries (timestamp_modification ${catkin_LIBRARIES}  ${Boost_LIBRARIES}) 




install(PROGRAMS
  scripts/keyboard_command.py
  scripts/rms.py
  scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(TARGETS aerial_robot_base_node aerial_robot_base
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


