cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_control)

add_compile_options(-std=c++11)

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


#list(APPEND CMAKE_PREFIX_PATH "~/acados/install/cmake")
find_package(acados) # if we use REQUIED, then will cause OpenBLAS and FortranLib Find_Package error


generate_dynamic_reconfigure_options(
  cfg/PID.cfg
  cfg/LQI.cfg
  cfg/NMPC.cfg
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
  LIBRARIES control_utils flight_control_pluginlib flight_navigation
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
# Create a list to hold the target names
set(ACADOS_PYTHON_TARGETS)

# Create a list of the scripts
set(ACADOS_PYTHON_SCRIPTS
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/fix_quad/nmpc_under_act_body_rate.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/fix_quad/nmpc_under_act_full.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_no_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo_dist.py
)

foreach(script ${ACADOS_PYTHON_SCRIPTS})
  message("Running ${script}...")
  get_filename_component(script_name ${script} NAME_WE)
  add_custom_target(
          ${script_name} ALL
          COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CATKIN_DEVEL_PREFIX}/lib/python3/dist-packages:$ENV{PYTHONPATH} ${Python3_EXECUTABLE} ${script}
          DEPENDS ${script} acados
          COMMENT "Running ${script}..."
  )
  # Append the target name to the list
  list(APPEND ACADOS_PYTHON_TARGETS ${script_name})
endforeach()

### flight control plugin
add_library(flight_control_pluginlib
  src/control/base/pose_linear_controller.cpp
  src/control/fully_actuated_controller.cpp
  src/control/under_actuated_controller.cpp
  src/control/under_actuated_lqi_controller.cpp
  src/control/under_actuated_tilted_lqi_controller.cpp
  src/nmpc/under_act_body_rate/mpc_solver.cpp
  src/nmpc/under_act_body_rate/nmpc_controller.cpp
  src/nmpc/under_act_full/mpc_solver.cpp
  src/nmpc/under_act_full/nmpc_controller.cpp
  src/nmpc/tilt_qd_no_servo_mdl/nmpc_solver.cpp
  src/nmpc/tilt_qd_no_servo_mdl/nmpc_controller.cpp
  src/nmpc/tilt_qd_servo_mdl/nmpc_solver.cpp
  src/nmpc/tilt_qd_servo_mdl/nmpc_controller.cpp
  src/nmpc/tilt_qd_servo_dist_mdl/nmpc_solver.cpp
  src/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller.cpp
  )
target_link_libraries(flight_control_pluginlib ${catkin_LIBRARIES} control_utils acados
        ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/under_act_body_rate/c_generated_code/libacados_ocp_solver_qd_body_rate_model.so
        ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/under_act_full/c_generated_code/libacados_ocp_solver_qd_full_model.so
        ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/tilt_qd_no_servo_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_no_servo_mdl.so
        ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/tilt_qd_servo_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_mdl.so
        ${PROJECT_SOURCE_DIR}/include/aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_dist_mdl.so
        )
add_dependencies(flight_control_pluginlib  ${PROJECT_NAME}_gencfg  ${ACADOS_PYTHON_TARGETS})

### flight navigation
add_library (flight_navigation src/flight_navigation.cpp)
target_link_libraries (flight_navigation ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS control_utils flight_control_pluginlib flight_navigation
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
