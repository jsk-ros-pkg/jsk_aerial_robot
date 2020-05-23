cmake_minimum_required(VERSION 2.8.3)
project(dragon)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hydrus
  aerial_robot_control
  aerial_robot_model
  aerial_robot_simulation
)

find_package(Eigen3 REQUIRED)

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

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES dragon_robot_model dragon_transform_control dragon_numerical_jacobians
  CATKIN_DEPENDS hydrus
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(dragon_robot_model src/dragon_robot_model.cpp)
target_link_libraries(dragon_robot_model ${catkin_LIBRARIES})

add_library(dragon_control_pluginlib src/lqi_gimbal_control.cpp)
target_link_libraries (dragon_control_pluginlib dragon_robot_model ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(dragon_control_pluginlib aerial_robot_msgs_generate_messages_cpp hydrus_gencfg)

# test
# pre-build test code
add_library(dragon_numerical_jacobians test/dragon/numerical_jacobians.cpp)
target_link_libraries(dragon_numerical_jacobians dragon_robot_model ${catkin_LIBRARIES})
add_executable(dragon_jacobian_test test/dragon/jacobian_test.cpp)
target_link_libraries(dragon_jacobian_test dragon_numerical_jacobians ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_subdirectory(test)
endif()
