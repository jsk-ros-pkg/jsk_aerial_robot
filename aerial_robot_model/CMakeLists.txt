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
  interactive_markers
  visualization_msgs
  tf
  tf2_ros
  tf2_kdl
  kdl_parser
)

find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rotor_interface
  CATKIN_DEPENDS roscpp  std_msgs  sensor_msgs geometry_msgs tf interactive_markers visualization_msgs
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


add_library(rotor_interface src/rotor_interface.cpp)
target_link_libraries(rotor_interface ${catkin_LIBRARIES})

add_executable(rotor_tf_publisher src/rotor_tf_publisher.cpp)
target_link_libraries(rotor_tf_publisher  ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(interactive_marker src/interactive_marker.cpp)
target_link_libraries(interactive_marker ${catkin_LIBRARIES})

add_executable(dragon2_joint_state_publisher src/dragon2_joint_state_publisher.cpp)
target_link_libraries(dragon2_joint_state_publisher ${catkin_LIBRARIES})

add_executable(hydrus_origin_publisher src/hydrus_origin_publisher.cpp)
target_link_libraries(hydrus_origin_publisher ${catkin_LIBRARIES})


add_executable(other_object_tf_publisher src/other_object_tf_publisher.cpp)
target_link_libraries(other_object_tf_publisher ${catkin_LIBRARIES})



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


## Mark executables and/or libraries for installation
install(TARGETS interactive_marker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

