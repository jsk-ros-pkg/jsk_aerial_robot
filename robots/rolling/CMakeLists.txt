cmake_minimum_required(VERSION 3.0.2)
project(rolling)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  mujoco_ros_control
  osqp_slsqp
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs osqp_slsqp pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  )

add_library(rolling_controller
  src/control/rolling_controller.cpp
  src/control/frame_control.cpp
  src/control/ground_controller.cpp
  src/control/pid_param_setter.cpp
  src/control/slsqp_solver.cpp)
target_link_libraries(rolling_controller ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(rolling_robot_model src/model/rolling_robot_model.cpp)
target_link_libraries(rolling_robot_model ${catkin_LIBRARIES})

add_library(rolling_navigation src/rolling_navigation.cpp)
target_link_libraries(rolling_navigation ${catkin_LIBRARIES})

add_executable(opt src/design/opt.cpp)
target_link_libraries(opt
  ${catkin_LIBRARIES})

add_executable(full_search src/design/full_search.cpp)
target_link_libraries(full_search
  ${catkin_LIBRARIES})

add_executable(fc_t_output src/design/fc_t_output.cpp)
target_link_libraries(fc_t_output
  ${catkin_LIBRARIES})

add_executable(optimal_design src/design/optimal_design.cpp)
target_link_libraries(optimal_design
  ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(mapping_optimization src/design/mapping_optimization.cpp)
target_link_libraries(mapping_optimization
  ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(pwm_republisher src/util/pwm_republisher.cpp)
target_link_libraries(pwm_republisher
  ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config launch plugins scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(TARGETS opt full_search fc_t_output optimal_design mapping_optimization pwm_republisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/mujoco
#   COMMAND rosrun aerial_robot_simulation mujoco_model_generator.py ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
#   POST_BUILD
#   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
#   DEPENDS ${PROJECT_SOURCE_DIR}/urdf
#           ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)
# add_custom_target(generate_mujoco_model ALL DEPENDS ${PROJECT_SOURCE_DIR}/mujoco)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)
