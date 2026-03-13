cmake_minimum_required(VERSION 3.0.2)
project(twin_hammer)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  diagnostic_msgs
  std_msgs
  geometry_msgs
  message_generation
  pluginlib
)

find_package(NLopt REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES twin_hammer_model twin_hammer_controller
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp diagnostic_msgs std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

# add_library(twin_hammer_controller src/control/twin_hammer_controller.cpp src/control/twin_hammer_optimizer.cpp)
# target_link_libraries(twin_hammer_controller ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})
# add_library(twin_hammer_controller src/control/twin_hammer_controller.cpp)
# target_link_libraries(twin_hammer_controller ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})
# add_library(twin_hammer_optimizer src/control/twin_hammer_optimizer.cpp)
# target_link_libraries(twin_hammer_optimizer ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})
add_library(twin_hammer_controllib src/control/twin_hammer_controller.cpp src/control/twin_hammer_optimizer.cpp)
target_link_libraries(twin_hammer_controllib ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(twin_hammer_model src/model/twin_hammer_model.cpp)
target_link_libraries(twin_hammer_model ${catkin_LIBRARIES})

# install(TARGETS twin_hammer_model twin_hammer_controller twin_hammer_optimizer
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
install(TARGETS twin_hammer_model twin_hammer_controllib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config plugins launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
