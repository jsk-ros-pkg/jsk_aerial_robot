cmake_minimum_required(VERSION 3.0.2)
project(ninja)

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
  beetle
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
  LIBRARIES ninja_robot_model ninja_controller ninja_navigation
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor diagnostic_msgs std_msgs geometry_msgs message_runtime beetle
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(ninja_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(ninja_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(ninja_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp ninja_generate_messages_cpp)

add_library(ninja_controller src/control/ninja_controller.cpp)
target_link_libraries(ninja_controller ninja_sensor_pluginlib ${catkin_LIBRARIES})

add_library(ninja_robot_model src/model/ninja_robot_model.cpp)
target_link_libraries(ninja_robot_model ${catkin_LIBRARIES})

add_library(ninja_navigation src/ninja_navigation.cpp)
target_link_libraries(ninja_navigation ${catkin_LIBRARIES})

install(TARGETS ninja_sensor_pluginlib ninja_robot_model ninja_controller ninja_navigation 
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

# catkin_install_python(PROGRAMS script/ninja/kondo_control.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )
