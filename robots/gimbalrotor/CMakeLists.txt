cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS
  script/arm_world_eval.py
  script/analyze_base_sway.py
  script/analyze_arm_world_limits.py
  script/render_arm_world_video.py
  script/render_arm_world_rviz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES script/arm_world_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  evaluation/base_sway/take9_2026-10-28-19-03-07_sway.npz
  evaluation/base_sway/take9_2026-10-28-19-03-07_summary.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/evaluation/base_sway
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
