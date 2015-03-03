cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()


#to find  eigen core for laser geometry
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


# find Eigen3
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(test_node src/test.cpp)
#rosbuild_add_executable(test_publish_node src/test_publish.cpp)

rosbuild_add_library (jsk_quadcopter src/quadcopter.cpp
  src/basic_controller_superior_controller_interface_lev0.cpp
  src/flight_navigation.cpp
  src/control_input_array.cpp
  src/flight_control.cpp
  src/state_estimation.cpp
  src/quadcopter_state_estimation.cpp
  src/kalman_filter.cpp
  src/digital_filter.cpp
  src/crc16.cpp
)

rosbuild_add_library (tracking src/tracking.cpp
  src/state_estimation.cpp
  src/flight_navigation.cpp
  src/control_input_array.cpp)


rosbuild_add_executable(jsk_quadcopter_node src/quadcopter_node.cpp)
target_link_libraries (jsk_quadcopter_node jsk_quadcopter tracking) 

#rosbuild_add_executable(tracking_node src/tracking_node.cpp)
#target_link_libraries (tracking_node jsk_quadcopter tracking) 
