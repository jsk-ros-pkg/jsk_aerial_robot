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
  dynamic_tf_publisher
  dynamic_reconfigure
  jsk_recognition_msgs #not good
  px_comm #not good
  #image_processing #not good
  aerial_robot_msgs
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
  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

generate_dynamic_reconfigure_options(
  cfg/PidPitchControl.cfg
  cfg/PidRollControl.cfg
  cfg/PidYawControl.cfg
  cfg/PidThrottleControl.cfg
  cfg/StateKalmanFilter.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp message_runtime aerial_robot_msgs std_msgs geometry_msgs visualization_msgs nav_msgs  sensor_msgs laser_geometry 
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)


add_library (aerial_robot_base src/aerial_robot_base.cpp
  src/flight_navigation.cpp
  src/flight_control.cpp
  src/state_estimation.cpp
  src/kalman_filter.cpp
)

add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
target_link_libraries (aerial_robot_base_node aerial_robot_base) 

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS aerial_robot_base_node aerial_robot_base
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


