cmake_minimum_required(VERSION 2.8.3)
project(dragon)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hydrus
  aerial_robot_control
  aerial_robot_model
  aerial_robot_msgs
)

find_package(Eigen3 REQUIRED)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

find_package(NLopt REQUIRED)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES dragon_robot_model dragon_aerial_robot_controllib dragon_navigation dragon_numerical_jacobians
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
  ${NLOPT_INCLUDE_DIRS}
)

# dragon euslisp model generation
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/quad/dragon.l
  COMMAND rosrun euscollada collada2eus dragon.dae dragon.yaml dragon.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots/quad
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/dragon.dae
          ${PROJECT_SOURCE_DIR}/robots/quad/dragon.yaml)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/quad/dragon.dae
  COMMAND rosrun collada_urdf urdf_to_collada dragon.urdf dragon.dae
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots/quad
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/dragon.urdf)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/quad/dragon.urdf
  COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} rosrun xacro xacro euclid_201709.urdf.xacro > dragon.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots/quad
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/euclid_201709.urdf.xacro
          ${PROJECT_SOURCE_DIR}/urdf/dragon_common.xacro
          ${PROJECT_SOURCE_DIR}/urdf/dragon_link.urdf.xacro)

# gte collada-dom version
find_package(PkgConfig)
pkg_check_modules(COLLADA collada-dom>=2.4.4)
if(${COLLADA_FOUND})
  add_custom_target(generate_dragon_default_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/dragon.l)
else()
  pkg_check_modules(COLLADA collada-dom)
  message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
endif()

add_library(dragon_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(dragon_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(dragon_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

add_library(dragon_robot_model src/model/hydrus_like_robot_model.cpp src/model/full_vectoring_robot_model.cpp)
target_link_libraries(dragon_robot_model ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(dragon_aerial_robot_controllib src/control/lqi_gimbal_control.cpp src/control/full_vectoring_control.cpp)
target_link_libraries (dragon_aerial_robot_controllib dragon_robot_model dragon_navigation dragon_sensor_pluginlib ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(dragon_aerial_robot_controllib aerial_robot_msgs_generate_messages_cpp hydrus_gencfg)

add_library(dragon_navigation src/dragon_navigation.cpp)
target_link_libraries(dragon_navigation ${catkin_LIBRARIES})


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
