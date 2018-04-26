cmake_minimum_required(VERSION 2.8.3)
project(hydrus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_base
  spinal
  dynamixel_controllers
  eigen_conversions
  tf_conversions
  cmake_modules
  std_srvs
  dynamic_reconfigure
  kdl_parser
  )

add_subdirectory(qpOASES)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/bin)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/include)
add_definitions(-DUSE_QPOASES)

find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_service_files(
  FILES
  AddExtraModule.srv
  TargetPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LQI.cfg)

catkin_package(
  INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/include
  LIBRARIES transform_control dynamixel_bridge
  CATKIN_DEPENDS roscpp message_runtime aerial_robot_msgs aerial_robot_base spinal dynamixel_controllers tf_conversions eigen_conversions kdl_parser
  DEPENDS Eigen3 orocos_kdl urdfdom_headers
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

cmake_policy(SET CMP0046 OLD)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
)

add_library (dynamixel_bridge src/dynamixel_bridge.cpp)
target_link_libraries (dynamixel_bridge ${catkin_LIBRARIES})
add_dependencies(dynamixel_bridge aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable(dynamixel_bridge_node src/dynamixel_bridge_node.cpp)
target_link_libraries (dynamixel_bridge_node  ${catkin_LIBRARIES} dynamixel_bridge)
add_dependencies(dynamixel_bridge_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library (transform_control src/transform_control.cpp)
target_link_libraries (transform_control ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES}  ${Eigen3_LIBRARIES} qpOASES)
add_dependencies(transform_control aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp spinal_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable (transform_control_node src/transform_control_node.cpp)
target_link_libraries (transform_control_node transform_control ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES}  ${Eigen3_LIBRARIES} qpOASES)
add_dependencies(transform_control_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp spinal_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

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


