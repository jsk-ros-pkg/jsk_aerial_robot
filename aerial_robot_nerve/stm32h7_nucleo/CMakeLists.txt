cmake_minimum_required(VERSION 3.0.2)
project(stm32h7_nucleo)

find_package(catkin REQUIRED COMPONENTS
  rosserial_msgs
  spinal
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  rosserial_msgs
  spinal
  std_msgs
)


add_custom_target(${PROJECT_NAME}_generate_ros_lib ALL COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh ${PYTHON_EXECUTABLE}  ${PROJECT_SOURCE_DIR}/scripts/make_libraries.py --save_path ${PROJECT_SOURCE_DIR} --support_rtos --support_ethernet)
add_dependencies(${PROJECT_NAME}_generate_ros_lib ${catkin_EXPORTED_TARGETS})

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
