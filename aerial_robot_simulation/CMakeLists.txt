cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_simulation)

add_compile_options(-std=c++11 ${GAZEBO_CXX_FLAGS})

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_estimation
  aerial_robot_msgs
  aerial_robot_model
  gazebo_ros_control
  kdl_parser
  roscpp
  spinal
  tf
  )

set(MUJOCO mujoco)
add_custom_command(OUTPUT ${MUJOCO}_build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND $(MAKE) mujoco
  VERBATIM
  )
add_custom_target(${MUJOCO} DEPENDS ${MUJOCO}_build)

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aerial_robot_hw_sim spinal_interface flight_controllers
  CATKIN_DEPENDS aerial_robot_base aerial_robot_estimation aerial_robot_msgs aerial_robot_model gazebo_ros_control kdl_parser roscpp spinal tf
  DEPENDS GAZEBO orocos_kdl urdfdom_headers
)

###########
## Build ##
###########

set(MUJOCO_ROOT_DIR ${PROJECT_SOURCE_DIR}/build/mujoco-2.3.7)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
  ${MUJOCO_ROOT_DIR}/include
  )

add_definitions(-DSIMULATION)

add_library(flight_controllers
  src/mujoco_attitude_controller.cpp
  src/simulation_attitude_controller.cpp)
add_dependencies(flight_controllers spinal_generate_messages_cpp ${MUJOCO})
target_link_libraries(flight_controllers ${catkin_LIBRARIES} ${MUJOCO_ROOT_DIR}/lib/libmujoco.so)

add_library(spinal_interface src/spinal_interface.cpp)
target_link_libraries(spinal_interface ${catkin_LIBRARIES})

add_library(mujoco_spinal_interface src/mujoco_spinal_interface.cpp)
add_dependencies(mujoco_spinal_interface ${MUJOCO})
target_link_libraries(mujoco_spinal_interface ${catkin_LIBRARIES} ${MUJOCO_ROOT_DIR}/lib/libmujoco.so)

add_library(aerial_robot_hw_sim src/aerial_robot_hw_sim.cpp)
target_link_libraries(aerial_robot_hw_sim spinal_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(robot_hw_sim src/robot_hw_sim.cpp)
add_dependencies(robot_hw_sim ${MUJOCO})
target_link_libraries(robot_hw_sim ${catkin_LIBRARIES} ${MUJOCO_ROOT_DIR}/lib/libmujoco.so)

add_library(mujoco_visualization_utils src/mujoco_visualization_utils.cpp)
add_dependencies(mujoco_visualization_utils ${MUJOCO})
target_link_libraries(mujoco_visualization_utils ${catkin_LIBRARIES} ${MUJOCO_ROOT_DIR}/lib/libmujoco.so)

add_library(mujoco_attitude_controller src/mujoco_attitude_controller.cpp)
add_dependencies(mujoco_attitude_controller ${MUJOCO})
target_link_libraries(mujoco_attitude_controller ${catkin_LIBRARIES} ${MUJOCO_ROOT_DIR}/lib/libmujoco.so)


add_executable(mujoco_ros src/mujoco_ros.cpp)
add_dependencies(mujoco_ros ${MUJOCO})
target_link_libraries(mujoco_ros
  ${catkin_LIBRARIES}
  glfw
  mujoco_visualization_utils
  mujoco_attitude_controller
  mujoco_spinal_interface
  robot_hw_sim
  ${MUJOCO_ROOT_DIR}/lib/libmujoco.so
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )


install(TARGETS aerial_robot_hw_sim spinal_interface flight_controllers mujoco_ros mujoco_visualization_utils mujoco_attitude_controller mujoco_spinal_interface robot_hw_sim
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES aerial_robot_hw_sim_plugins.xml flight_controllers_plugins.xml robot_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
