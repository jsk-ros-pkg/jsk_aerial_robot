cmake_minimum_required(VERSION 3.0.2)
project(hydrus)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  spinal
  eigen_conversions
  tf_conversions
  std_srvs
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES hydrus_robot_model hydrus_controller_pluginlib hydrus_numerical_jacobians
  CATKIN_DEPENDS roscpp aerial_robot_msgs aerial_robot_control aerial_robot_model spinal eigen_conversions tf_conversions std_srvs
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
  test
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(hydrus_robot_model
  src/hydrus_robot_model.cpp
  src/hydrus_tilted_robot_model.cpp)
target_link_libraries(hydrus_robot_model ${catkin_LIBRARIES})

### flight control plugin
add_library(hydrus_controller_pluginlib
  src/hydrus_lqi_controller.cpp
  src/hydrus_tilted_lqi_controller.cpp)
target_link_libraries(hydrus_controller_pluginlib hydrus_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(hydrus_controller_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

# test
# pre-build test code
add_library(hydrus_numerical_jacobians test/hydrus/numerical_jacobians.cpp)
target_link_libraries(hydrus_numerical_jacobians hydrus_robot_model ${catkin_LIBRARIES})
add_executable(hydrus_jacobian_test test/hydrus/jacobian_test.cpp)
target_link_libraries(hydrus_jacobian_test hydrus_numerical_jacobians ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/ test/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS hydrus_jacobian_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hydrus_robot_model hydrus_controller_pluginlib hydrus_numerical_jacobians
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config scripts plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()

