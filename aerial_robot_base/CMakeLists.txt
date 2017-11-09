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
  serial
  geodesy
  mavros_msgs
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_message_files(FILES
  AerialRobotStatus.msg
  FlightNav.msg
  FlatnessPid.msg
  Pid.msg
  State.msg
  States.msg
  Pwms.msg
  DesireCoord.msg
  Acc.msg
  UavInfo.msg
  )

add_service_files(
  FILES
  GetMotorNum.srv
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs)

generate_dynamic_reconfigure_options(
  cfg/XYPidControl.cfg
  cfg/KalmanFilterBaroBias.cfg
  cfg/KalmanFilterXYBias.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} flight_control_pluginlib
  CATKIN_DEPENDS roscpp message_runtime aerial_robot_msgs std_msgs geometry_msgs visualization_msgs nav_msgs  sensor_msgs laser_geometry tf kalman_filter pluginlib geodesy mavros_msgs
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

### sensor plugin
add_library(sensor_pluginlib
  src/sensor/optical_flow.cpp
  src/sensor/vo.cpp
  src/sensor/altitude.cpp
  src/sensor/mocap.cpp
  src/sensor/gps.cpp
  src/sensor/imu.cpp)
target_link_libraries(sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(sensor_pluginlib aerial_robot_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

### kf plugin
add_library(kf_baro_bias_pluginlib
  src/kf/baro_bias_plugin.cpp
  src/kf/xy_roll_pitch_bias_plugin.cpp)
target_link_libraries(kf_baro_bias_pluginlib ${catkin_LIBRARIES})

### flight control plugin
add_library(flight_control_pluginlib
  src/control/flatness_pid_controller.cpp)
target_link_libraries(flight_control_pluginlib ${catkin_LIBRARIES})
add_dependencies(flight_control_pluginlib aerial_robot_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library (aerial_robot_base src/aerial_robot_base.cpp
  src/flight_navigation.cpp
  src/state_estimation.cpp
)
add_dependencies(aerial_robot_base aerial_robot_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries (aerial_robot_base ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(aerial_robot_base aerial_robot_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
target_link_libraries (aerial_robot_base_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} aerial_robot_base)
add_dependencies(aerial_robot_base_node aerial_robot_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)


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


