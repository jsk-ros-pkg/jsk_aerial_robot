cmake_minimum_required(VERSION 3.0.2)
project(mini_quadrotor)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/mujoco
  COMMAND rosrun mujoco_ros_control mujoco_model_generator.py ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
  POST_BUILD
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  DEPENDS ${PROJECT_SOURCE_DIR}/urdf
          ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

add_custom_target(generate_mujoco_model ALL DEPENDS ${PROJECT_SOURCE_DIR}/mujoco)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
