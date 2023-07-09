cmake_minimum_required(VERSION 3.0.2)
project(rolling)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  hydrus_xi
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs hydrus_xi pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  )

add_library(rolling_controller src/control/rolling_controller.cpp)
target_link_libraries(rolling_controller ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(rolling_robot_model src/model/rolling_robot_model.cpp)
target_link_libraries(rolling_robot_model ${catkin_LIBRARIES})

add_library(rolling_navigation src/rolling_navigation.cpp)
target_link_libraries(rolling_navigation ${catkin_LIBRARIES})

add_executable(opt src/design/opt.cpp)
target_link_libraries(opt
  ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_executable(optimal_design src/design/optimal_design.cpp)
target_link_libraries(optimal_design
  ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

install(DIRECTORY config launch plugins scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
