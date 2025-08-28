cmake_minimum_required(VERSION 3.0.2)
project(hugmy)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  pluginlib
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  jsk_recognition_msgs
  aerial_robot_msgs
  cv_bridge
  tf
  image_transport
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control pluginlib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(hugmy_navigation src/navigation/hugmy_navigation.cpp)
target_link_libraries(hugmy_navigation ${catkin_LIBRARIES})

add_executable(haptics_integrated_controller src/control/integrated.cpp
  src/control/air_pressure_controller.cpp
  src/control/haptics_controller.cpp
  src/control/rviz_visualizer.cpp
  )

target_link_libraries(haptics_integrated_controller ${catkin_LIBRARIES})
add_dependencies(haptics_integrated_controller ${catkin_EXPORTED_TARGETS})

add_executable(perching_node src/control/perching.cpp)
target_link_libraries(perching_node ${catkin_LIBRARIES})
add_dependencies(perching_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config launch plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

