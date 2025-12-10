cmake_minimum_required(VERSION 3.0.2)
project(crobat)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_control
  gimbalrotor
  aerial_robot_control
  aerial_robot_model
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model gimbalrotor
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
add_library(crobat_robot_model src/model/crobat_robot_model.cpp)
target_link_libraries(crobat_robot_model ${catkin_LIBRARIES})

add_library(crobat_controller src/control/crobat_controller.cpp)
target_link_libraries(crobat_controller ${catkin_LIBRARIES} crobat_robot_model)

add_library(crobat_navigation src/crobat_navigation.cpp)
target_link_libraries(crobat_navigation ${catkin_LIBRARIES} crobat_robot_model)


mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
