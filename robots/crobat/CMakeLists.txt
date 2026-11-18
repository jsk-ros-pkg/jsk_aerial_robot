cmake_minimum_required(VERSION 3.0.2)
project(crobat)
add_compile_options(-std=c++17)

option(CROBAT_BUILD_MUJOCO_MODEL
  "Generate the Crobat MuJoCo model while building this package" ON)

set(CROBAT_CATKIN_COMPONENTS
  gimbalrotor
  aerial_robot_control
  aerial_robot_model
)

if(CROBAT_BUILD_MUJOCO_MODEL)
  list(APPEND CROBAT_CATKIN_COMPONENTS mujoco_ros_control)
endif()

find_package(catkin REQUIRED COMPONENTS ${CROBAT_CATKIN_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model gimbalrotor
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS
  scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(crobat_robot_model src/model/crobat_robot_model.cpp)
target_link_libraries(crobat_robot_model ${catkin_LIBRARIES})

add_library(crobat_controller src/control/crobat_controller.cpp)
target_link_libraries(crobat_controller ${catkin_LIBRARIES} crobat_robot_model)

add_library(crobat_navigation src/crobat_navigation.cpp)
target_link_libraries(crobat_navigation ${catkin_LIBRARIES} crobat_robot_model)


if(CROBAT_BUILD_MUJOCO_MODEL)
  mujoco_model_convert(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
