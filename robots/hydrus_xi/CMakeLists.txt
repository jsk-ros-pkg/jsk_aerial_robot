cmake_minimum_required(VERSION 3.0.2)
project(hydrus_xi)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  angles
  hydrus
  mujoco_ros_control
  roscpp
  )

find_package(NLopt REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  hydrus_xi_fully_actuated_robot_model
  CATKIN_DEPENDS  angles hydrus roscpp
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

add_library(hydrus_xi_under_actuated_navigation src/hydrus_xi_under_actuated_navigation.cpp)
target_link_libraries(hydrus_xi_under_actuated_navigation ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(hydrus_xi_fully_actuated_robot_model src/hydrus_xi_fully_actuated_robot_model.cpp)
target_link_libraries(hydrus_xi_fully_actuated_robot_model ${catkin_LIBRARIES})

# pre-build test code
add_executable(hydrus_xi_jacobian_test test/hydrus_xi/jacobian_test.cpp)
target_link_libraries(hydrus_xi_jacobian_test hydrus_xi_fully_actuated_robot_model ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS hydrus_xi_jacobian_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hydrus_xi_under_actuated_navigation hydrus_xi_fully_actuated_robot_model
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config scripts plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES flight_navigation_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
