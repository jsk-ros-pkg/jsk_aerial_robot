cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_estimation)

add_compile_options(-std=c++17)

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
  pluginlib
  sensor_msgs
  spinal
  tf
  tf_conversions
  jsk_recognition_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/KalmanFilterXYBias.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} sensor_pluginlib
  CATKIN_DEPENDS aerial_robot_model aerial_robot_msgs cv_bridge dynamic_reconfigure geodesy geographic_msgs geometry_msgs kalman_filter nav_msgs nodelet pluginlib sensor_msgs spinal tf tf_conversions jsk_recognition_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### state estimation
add_library(aerial_robot_estimation
  src/state_estimation.cpp)
target_link_libraries(aerial_robot_estimation ${catkin_LIBRARIES})

### sensor plugins
add_library(sensor_pluginlib
  src/sensor/vo.cpp
  src/sensor/altitude.cpp
  src/sensor/mocap.cpp
  src/sensor/gps.cpp
  src/sensor/imu.cpp
  src/sensor/plane_detection.cpp)

target_link_libraries(sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

### kalman filter plugins
add_library(kf_baro_bias_pluginlib
  src/kf/baro_bias_plugin.cpp
  src/kf/xy_roll_pitch_bias_plugin.cpp)
target_link_libraries(kf_baro_bias_pluginlib ${catkin_LIBRARIES})
add_dependencies(kf_baro_bias_pluginlib ${PROJECT_NAME}_gencfg)

set(USE_GPU OFF)

if(USE_GPU)
  add_definitions(-DUSE_GPU)
endif()

add_library(optical_flow src/vision/optical_flow.cpp)
target_link_libraries(optical_flow ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS aerial_robot_estimation sensor_pluginlib kf_baro_bias_pluginlib optical_flow
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch plugins config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
