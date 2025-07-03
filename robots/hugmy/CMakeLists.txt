cmake_minimum_required(VERSION 3.0.2)
project(hugmy)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  pluginlib
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

add_executable(haptics_integrated_controller src/control/pwm_sender.cpp
  src/control/air_pressure_controller.cpp
  src/control/haptics_controller.cpp
  )
target_link_libraries(haptics_integrated_controller ${catkin_LIBRARIES})
add_dependencies(haptics_integrated_controller ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config launch plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

