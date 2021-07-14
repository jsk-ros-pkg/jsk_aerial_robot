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
  dynamic_reconfigure
  )

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LQI.cfg)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES hydrus_robot_model hydrus_controller_pluginlib hydrus_numerical_jacobians
  CATKIN_DEPENDS roscpp aerial_robot_msgs aerial_robot_control aerial_robot_model spinal eigen_conversions tf_conversions std_srvs dynamic_reconfigure
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

# hydrus euslisp model generation
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.l
  COMMAND rosrun euscollada collada2eus hydrus.dae hydrus.yaml hydrus.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.dae
          ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.yaml)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.dae
  COMMAND rosrun collada_urdf urdf_to_collada hydrus.urdf hydrus.dae
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.urdf)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.urdf
  COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} rosrun xacro xacro robot.urdf.xacro > hydrus.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/robot.urdf.xacro
          ${PROJECT_SOURCE_DIR}/urdf/common.xacro
          ${PROJECT_SOURCE_DIR}/urdf/link.urdf.xacro)

# gte collada-dom version
find_package(PkgConfig)
pkg_check_modules(COLLADA collada-dom>=2.4.4)
if(${COLLADA_FOUND})
  add_custom_target(generate_hydrus_default_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/robots/quad/default_mode_201907/hydrus.l)
else()
  pkg_check_modules(COLLADA collada-dom)
  message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
endif()

add_library(hydrus_robot_model
  src/hydrus_robot_model.cpp
  src/hydrus_tilted_robot_model.cpp)
target_link_libraries(hydrus_robot_model ${catkin_LIBRARIES})

### flight control plugin
add_library(hydrus_controller_pluginlib
  src/hydrus_lqi_controller.cpp
  src/hydrus_tilted_lqi_controller.cpp)
target_link_libraries(hydrus_controller_pluginlib hydrus_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(hydrus_controller_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp ${PROJECT_NAME}_gencfg)

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

