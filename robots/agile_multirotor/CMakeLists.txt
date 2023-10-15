cmake_minimum_required(VERSION 2.8.3)
project(agile_multirotor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  eigen_conversions
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

catkin_package(
  CATKIN_DEPENDS roscpp aerial_robot_control aerial_robot_model aerial_robot_msgs
)

###########
## Build ##
###########
add_executable(collision_detection_node src/collision_detection.cpp)
target_link_libraries(collision_detection_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(observation_conversion_node src/observation_cal.cpp)
target_link_libraries(observation_conversion_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(common_observation_conversion_node src/common_observation_cal.cpp)
target_link_libraries(common_observation_conversion_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})