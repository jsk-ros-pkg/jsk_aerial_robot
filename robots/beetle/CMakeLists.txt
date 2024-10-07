cmake_minimum_required(VERSION 3.0.2)
project(beetle)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  gimbalrotor
  diagnostic_msgs
  std_msgs
  geometry_msgs
  message_generation
  )

find_package(Eigen3 REQUIRED)

add_message_files(FILES
  TaggedWrench.msg
  )

catkin_python_setup()

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beetle_robot_model beetle_controller beetle_navigation
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor diagnostic_msgs std_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(beetle_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(beetle_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(beetle_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp beetle_generate_messages_cpp)

add_library(beetle_controller src/control/beetle_controller.cpp)
target_link_libraries(beetle_controller beetle_sensor_pluginlib ${catkin_LIBRARIES})

add_library(beetle_robot_model src/model/beetle_robot_model.cpp)
target_link_libraries(beetle_robot_model ${catkin_LIBRARIES})

add_library(beetle_navigation src/beetle_navigation.cpp)
target_link_libraries(beetle_navigation ${catkin_LIBRARIES})

install(TARGETS beetle_sensor_pluginlib beetle_robot_model beetle_controller beetle_navigation 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# catkin_install_python(PROGRAMS script/beetle/kondo_control.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )
