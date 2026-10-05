cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_control)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  dynamic_reconfigure
  pluginlib
  roscpp
  spinal
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PythonInterp REQUIRED)

# TODO: auto install acados
find_package(acados) # if we use REQUIRED, then will cause OpenBLAS and FortranLib Find_Package error

set(ACADOS_GENERATED_LIBS
  ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/fix_qd_thrust_out_mdl/c_generated_code/libacados_ocp_solver_fix_qd_thrust_out_mdl.so
  ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_dist_mdl.so
  ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/c_generated_code/libacados_sim_solver_tilt_qd_servo_dist_mdl.so
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
  cfg/LQI.cfg
  cfg/NMPC.cfg
  cfg/ITerm.cfg
  )

### Temporary soulition because of https://github.com/ros/ros_comm/issues/1404
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)|(armhf)" AND $ENV{ROS_DISTRO} MATCHES "melodic")
  message(WARNING "please check https://github.com/ros/ros_comm/issues/1404")
  add_definitions (-DARM_MELODIC)
endif()


# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_utils flight_control_pluginlib flight_navigation trajectory_generation mpc_solver_plugins
  CATKIN_DEPENDS aerial_robot_estimation aerial_robot_model aerial_robot_msgs dynamic_reconfigure pluginlib roscpp spinal tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

### control utils
add_library(control_utils
  src/control/utils/care.cpp
  )
target_link_libraries(control_utils ${EIGEN3_LIBRARIES})

### acados code generation
set(PYTHON_REQUIREMENTS_STAMP ${CMAKE_CURRENT_BINARY_DIR}/python_requirements.stamp)
add_custom_command(
  OUTPUT ${PYTHON_REQUIREMENTS_STAMP}
  COMMAND ${PYTHON_EXECUTABLE} -m pip install --user -r ${PROJECT_SOURCE_DIR}/scripts/requirements.txt
  COMMAND ${CMAKE_COMMAND} -E touch ${PYTHON_REQUIREMENTS_STAMP}
  DEPENDS ${PROJECT_SOURCE_DIR}/scripts/requirements.txt
  COMMENT "Installing Python requirements for NMPC code generation"
)

add_custom_target(install_python_requirements
  DEPENDS ${PYTHON_REQUIREMENTS_STAMP}
)

add_custom_target(gen_acados_code ALL
  COMMAND bash ${PROJECT_SOURCE_DIR}/scripts/nmpc/gen_nmpc_code_all.sh
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/scripts/nmpc/
  COMMENT "Generating acados code for NMPC models"
)
add_dependencies(gen_acados_code install_python_requirements)

### mpc solver plugin
add_library(mpc_solver_plugins src/nmpc/mpc_solver_plugins.cpp)
target_link_libraries(mpc_solver_plugins ${catkin_LIBRARIES} acados ${ACADOS_GENERATED_LIBS})
add_dependencies(mpc_solver_plugins gen_acados_code)

### flight control plugin
add_library(flight_control_pluginlib
  src/control/base/pose_linear_controller.cpp
  src/control/fully_actuated_controller.cpp
  src/control/under_actuated_controller.cpp
  src/control/under_actuated_lqi_controller.cpp
  src/control/under_actuated_tilted_lqi_controller.cpp
  src/nmpc/base_mpc_controller.cpp
  src/nmpc/fix_qd_thrust_out_mdl/nmpc_controller.cpp
  src/nmpc/tilt_mt_servo_nmpc_controller.cpp
  src/nmpc/tilt_mt_servo_dist_nmpc_controller.cpp
)

target_link_libraries(flight_control_pluginlib ${catkin_LIBRARIES} control_utils mpc_solver_plugins)
add_dependencies(flight_control_pluginlib  ${PROJECT_NAME}_gencfg)

### flight navigation
add_library (flight_navigation src/flight_navigation.cpp src/util/joy_parser.cpp)
target_link_libraries (flight_navigation ${catkin_LIBRARIES})

### trajectory generate based on dodgelib
add_library (trajectory_generation
  src/trajectory/reference_base.cpp
  src/trajectory/trajectory_reference/polynomial.cpp
  src/trajectory/trajectory_reference/polynomial_trajectory.cpp
  src/trajectory/trajectory_reference/sampled_trajectory.cpp
  src/trajectory/base/parameter_base.cpp
  src/trajectory/math/math.cpp
  src/trajectory/types/command.cpp
  src/trajectory/types/quad_state.cpp
  src/trajectory/types/quadrotor.cpp
  src/trajectory/utils/logger.cpp
  src/trajectory/utils/timer.cpp
  )
target_link_libraries (trajectory_generation ${catkin_LIBRARIES})

add_executable(trajectory_generation_node src/trajectory/test.cpp)
target_link_libraries (trajectory_generation_node ${catkin_LIBRARIES} trajectory_generation)



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "tilt_qd_servo_mdl" EXCLUDE
  PATTERN "*.so" EXCLUDE)

install(TARGETS control_utils mpc_solver_plugins flight_control_pluginlib flight_navigation trajectory_generation
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS trajectory_generation_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${ACADOS_GENERATED_LIBS}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  OPTIONAL
)

install(DIRECTORY scripts plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
