cmake_minimum_required(VERSION 2.8.3)
project(dragon)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  hydrus
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
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
  INCLUDE_DIRS include
  LIBRARIES dragon
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib Eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dragon_control
  src/dragon_control.cpp
  )
target_link_libraries (dragon_control ${catkin_LIBRARIES})
add_dependencies(dragon_control aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp hydrus_generate_messages_cpp hydrus_gencfg)


add_executable (dragon_control_node src/dragon_control_node.cpp)
target_link_libraries (dragon_control_node ${catkin_LIBRARIES} dragon_control)
add_dependencies(dragon_control_node aerial_robot_msgs_generate_messages_cpp aerial_robot_base_generate_messages_cpp hydrus_generate_messages_cpp hydrus_gencfg)
