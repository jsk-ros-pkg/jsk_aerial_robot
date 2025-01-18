cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_base)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  roscpp
  rospy)

catkin_python_setup()

add_compile_options(-std=c++17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library (aerial_robot_base src/aerial_robot_base.cpp)
target_link_libraries (aerial_robot_base ${catkin_LIBRARIES})

add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
target_link_libraries (aerial_robot_base_node ${catkin_LIBRARIES} aerial_robot_base)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS aerial_robot_base
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS aerial_robot_base_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
