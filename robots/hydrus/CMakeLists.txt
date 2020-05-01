cmake_minimum_required(VERSION 2.8.3)
project(hydrus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_base
  aerial_robot_control
  aerial_robot_model
  spinal
  eigen_conversions
  tf_conversions
  std_srvs
  dynamic_reconfigure
  )

find_package(Eigen3 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

generate_dynamic_reconfigure_options(
  cfg/LQI.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_robot_model transform_control
  CATKIN_DEPENDS roscpp aerial_robot_msgs aerial_robot_base aerial_robot_model spinal tf_conversions eigen_conversions
  DEPENDS EIGEN3
)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(hydrus_robot_model
  src/hydrus_robot_model.cpp
  src/tilted_robot_model.cpp)
target_link_libraries(hydrus_robot_model ${catkin_LIBRARIES})

add_library(transform_control src/transform_control.cpp)
target_link_libraries(transform_control hydrus_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(transform_control aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable(transform_control_node src/transform_control_node.cpp)
target_link_libraries(transform_control_node transform_control ${catkin_LIBRARIES}  ${EIGEN3_LIBRARIES})

add_library(tilted_transform_control src/tilted_transform_control.cpp)
target_link_libraries(tilted_transform_control hydrus_robot_model transform_control ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(tilted_transform_control_node src/tilted_transform_control_node.cpp)
target_link_libraries(tilted_transform_control_node tilted_transform_control ${catkin_LIBRARIES}  ${EIGEN3_LIBRARIES})



install(PROGRAMS
  scripts/dynamixel_param.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS transform_control tilted_transform_control
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# test
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_subdirectory(test)
endif()

