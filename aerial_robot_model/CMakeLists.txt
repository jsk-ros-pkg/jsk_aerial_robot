cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  std_msgs  sensor_msgs geometry_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

add_library(joint_state_publisher src/joint_state_publisher.cpp)
add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp)
target_link_libraries(joint_state_publisher_node joint_state_publisher ${catkin_LIBRARIES})


add_executable(traj_motion_node src/traj_motion.cpp)
target_link_libraries(traj_motion_node ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


## Mark executables and/or libraries for installation
install(TARGETS joint_state_publisher joint_state_publisher traj_motion_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

