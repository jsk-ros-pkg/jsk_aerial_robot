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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
  )

add_definitions(-DSIMULATION)

add_library(flight_controllers
  src/simulation_attitude_controller.cpp)
target_link_libraries(flight_controllers ${catkin_LIBRARIES})
add_dependencies(flight_controllers spinal_generate_messages_cpp)

add_library(spinal_interface src/spinal_interface.cpp)
target_link_libraries(spinal_interface ${catkin_LIBRARIES})

add_library(aerial_robot_hw_sim src/aerial_robot_hw_sim.cpp)
target_link_libraries(aerial_robot_hw_sim spinal_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS aerial_robot_hw_sim spinal_interface flight_controllers
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES aerial_robot_hw_sim_plugins.xml flight_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
