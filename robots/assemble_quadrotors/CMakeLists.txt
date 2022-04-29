cmake_minimum_required(VERSION 2.8.3)
project(assemble_quadrotors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  )

find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

catkin_package(
  CATKIN_DEPENDS roscpp aerial_robot_control aerial_robot_model aerial_robot_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########
add_executable(optimal_design_node src/optimal_design.cpp)
target_link_libraries(optimal_design_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})
