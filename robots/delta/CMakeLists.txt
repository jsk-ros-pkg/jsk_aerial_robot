cmake_minimum_required(VERSION 3.0.2)
project(delta)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  dynamic_reconfigure
  geometry_msgs
  hydrus
  message_generation
  mujoco_ros_control
  pluginlib
  roscpp
  std_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(OsqpEigen REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

catkin_python_setup()

add_message_files(FILES
  DynamicReconfigureLevels.msg
  )
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

generate_dynamic_reconfigure_options(
  cfg/nlopt.cfg
  )
add_dependencies(${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_py)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs dynamic_reconfigure geometry_msgs hydrus message_runtime pluginlib roscpp std_msgs
  LIBRARIES delta_controller delta_robot_model delta_navigation
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  )

add_library(delta_controller
  src/control/delta_controller_common.cpp
  src/control/delta_controller_utils.cpp
  src/control/delta_flight_controller.cpp
  src/control/delta_ground_controller.cpp
  src/control/delta_joint_controller.cpp
  )
target_link_libraries(delta_controller ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES} ${NLOPT_LIBRARIES})
add_dependencies(delta_controller ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(delta_robot_model src/model/delta_robot_model.cpp)
target_link_libraries(delta_robot_model ${catkin_LIBRARIES})

add_library(delta_navigation
  src/navigation/delta_locomotion_planner.cpp
  src/navigation/delta_navigation.cpp
  src/navigation/delta_navigation_utils.cpp
  src/navigation/delta_transform_planner.cpp)
target_link_libraries(delta_navigation ${catkin_LIBRARIES})

add_executable(traj_test src/navigation/traj_test.cpp)
target_link_libraries(traj_test ${catkin_LIBRARIES})

add_executable(delta_optimal_design src/design/delta_optimal_design.cpp)
target_link_libraries(delta_optimal_design ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(delta_joint_load_calc src/design/delta_joint_load_calc.cpp)
target_link_libraries(delta_joint_load_calc
  ${catkin_LIBRARIES} delta_robot_model)

add_executable(opt src/design/opt.cpp)
target_link_libraries(opt ${catkin_LIBRARIES})

add_executable(full_search src/design/full_search.cpp)
target_link_libraries(full_search ${catkin_LIBRARIES})

add_executable(fc_t_output src/design/fc_t_output.cpp)
target_link_libraries(fc_t_output ${catkin_LIBRARIES})

add_executable(optimal_design src/design/optimal_design.cpp)
target_link_libraries(optimal_design ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(mapping_optimization src/design/mapping_optimization.cpp)
target_link_libraries(mapping_optimization ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(pwm_republisher src/util/pwm_republisher.cpp)
target_link_libraries(pwm_republisher ${catkin_LIBRARIES} delta_robot_model)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config launch plugins scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(TARGETS delta_joint_load_calc delta_optimal_design opt full_search fc_t_output optimal_design mapping_optimization pwm_republisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS delta_controller delta_robot_model delta_navigation
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

catkin_add_env_hooks(99.delta SHELLS bash sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()

