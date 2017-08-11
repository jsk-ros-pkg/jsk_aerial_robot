cmake_minimum_required(VERSION 2.8.3)
project(hydrus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_base
  dynamixel_controllers
  eigen_conversions
  tf_conversions
  cmake_modules
  std_srvs
  dynamic_reconfigure
  kdl_parser
  )

find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_message_files(
  FILES
  ServoState.msg
  ServoStates.msg
  ServoControl.msg
  Gyro.msg
)

add_service_files(
  FILES
  AddExtraModule.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LQI.cfg)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES transform_control
  CATKIN_DEPENDS roscpp message_runtime aerial_robot_msgs dynamixel_controllers tf_conversions
  DEPENDS Eigen
)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
message(STATUS "CMAKE_BUILD_TYPE:               ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE:        ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_CXX_FLAGS_MINSIZEREL:     ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:          ${CMAKE_CXX_FLAGS_DEBUG}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

add_executable(dynamicxel_bridge_node src/dynamicxel_bridge.cpp)
target_link_libraries (dynamicxel_bridge_node  ${catkin_LIBRARIES})
add_dependencies(dynamicxel_bridge_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library (transform_control src/transform_control.cpp)
target_link_libraries (transform_control ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(transform_control aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable (transform_control_node src/transform_control_node.cpp)
target_link_libraries (transform_control_node transform_control ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(transform_control_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

install(PROGRAMS
  scripts/dynamixel_param.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS transform_control transform_control_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


