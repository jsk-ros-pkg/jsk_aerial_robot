cmake_minimum_required(VERSION 3.0.2)
project(hydrus_gripper)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  hydrus
  )


catkin_package(
  CATKIN_DEPENDS hydrus
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
