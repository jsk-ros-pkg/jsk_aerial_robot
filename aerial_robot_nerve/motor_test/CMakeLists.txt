cmake_minimum_required(VERSION 3.0.2)
project(motor_test)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  takasako_sps
  spinal
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs takasako_sps spinal
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_test_node src/motor_test.cpp)

target_link_libraries(motor_test_node
   ${catkin_LIBRARIES}
 )

install(TARGETS  motor_test_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
