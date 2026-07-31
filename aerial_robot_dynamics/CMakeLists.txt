cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_dynamics)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(urdfdom_headers REQUIRED)

option(BUILD_BENCH "Build Benchmark Code" OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  CATKIN_DEPENDS roscpp urdf
  CFG_EXTRAS aerial_robot_dynamics-extras.cmake
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  )

add_library(robot_model src/robot_model.cpp)
target_link_libraries(robot_model
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  OsqpEigen::OsqpEigen
  )

install(TARGETS robot_model
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_BENCH)
  add_subdirectory(benchmark)
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
