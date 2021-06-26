cmake_minimum_required(VERSION 2.8.3)
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
