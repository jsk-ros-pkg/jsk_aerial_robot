cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_estimation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_model
  aerial_robot_msgs
  cv_bridge
  dynamic_reconfigure
  geodesy
  geographic_msgs
  geometry_msgs
  kalman_filter
  nav_msgs
  nodelet
  roscpp
  rospy
  sensor_msgs
  spinal
  std_msgs
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/KalmanFilterXYBias.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} sensor_pluginlib
  CATKIN_DEPENDS geodesy kalman_filter nodelet spinal tf tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### state estimation
add_library(aerial_robot_estimation
  src/state_estimation)
target_link_libraries(aerial_robot_estimation ${catkin_LIBRARIES})

### sensor plugins
add_library(sensor_pluginlib
  src/sensor/vo.cpp
  src/sensor/altitude.cpp
  src/sensor/mocap.cpp
  src/sensor/gps.cpp
  src/sensor/imu.cpp)
target_link_libraries(sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

### kalman filter plugins
add_library(kf_baro_bias_pluginlib
  src/kf/baro_bias_plugin.cpp
  src/kf/xy_roll_pitch_bias_plugin.cpp)
target_link_libraries(kf_baro_bias_pluginlib ${catkin_LIBRARIES})
add_dependencies(kf_baro_bias_pluginlib ${PROJECT_NAME}_gencfg)

set(USE_GPU OFF)
set(CMAKE_BUILD_TYPE Release)

if(USE_GPU)
  add_definitions(-DUSE_GPU)
endif()

add_library(optical_flow src/vision/optical_flow.cpp)
target_link_libraries(optical_flow ${catkin_LIBRARIES})
